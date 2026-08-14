# SPDX-License-Identifier: MulanPSL-2.0
"""rclpy node: pose + lidar + /map in, /cmd_vel + /simple_nav/path out.

Architecture:
  /map (OccupancyGrid)          ─┐
  /odom + /amcl_pose (pose)      ├──► state machine: when a new goal
  /scanner (lidar clearance)    ─┘    arrives, replan A* once and feed
                                       the resulting path into the RPP
                                       follower at 10 Hz.
  /goal_pose (rviz "2D Nav Goal") triggers a new goal directly.
  /cmd_vel + /simple_nav/path are the published outputs.

Goal lifecycle: active → succeeded / aborted / cancelled. Status is
read by the MCP `status` tool; cancellation by `cancel`.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

from . import planner as _planner
from . import follower as _follower


# ── ROS imports lazily (so unit tests don't drag rclpy) ─────────────
def _import_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan, Image
    from nav_msgs.msg import Odometry, OccupancyGrid, Path
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
    from std_msgs.msg import Header
    from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
    return locals()


@dataclass
class Goal:
    goal_id: str
    target_x: float
    target_y: float
    # Target heading at goal. None = whatever yaw the robot ends up
    # with after pure-pursuit (typically along the last path segment);
    # set to a number = drive to (x,y) then rotate-in-place to align.
    # rviz "2D Nav Goal" tool always provides one (the arrow you
    # drag), MCP/gRPC callers may pass None to stay yaw-agnostic.
    target_yaw: Optional[float] = None
    yaw_tolerance_rad: float = 0.10  # ~6°
    tolerance_m: float = 0.1
    started_at: float = field(default_factory=time.time)
    # Sub-state for two-phase goals: "approach" (driving xy)  →
    # "align" (rotating to target_yaw) → "succeeded".
    phase: str = "approach"
    state: str = "active"   # active | succeeded | aborted | cancelled
    detail: str = ""
    # Cached plan (world (x,y) waypoints) — set when planner succeeds.
    path: Optional[List[Tuple[float, float]]] = None
    plan_attempts: int = 0
    # Costmap layers from the latest plan_world() call. Follower
    # samples ``cost`` at the robot's pose to slow down near walls and
    # ``passable`` to detect "we're inside the inscribed halo, abort".
    costmap: Optional[object] = None  # PlanResult, kept loose to avoid forward-import
    # Map version (NavNode._map_counter) the cached `path` was planned
    # against. When _map_counter advances past this, the next tick
    # re-validates the path against the fresh map; if any remaining
    # waypoint lies in a now-impassable cell, the path is invalidated
    # and the next tick replans. Without this the robot follows a
    # stale plan straight into newly-discovered obstacles until the
    # stuck detector aborts the goal eight seconds later.
    plan_map_counter: int = -1
    # Stuck-detector state. Pose snapshots tagged with wall-clock time;
    # if we've moved < `stuck_progress_m` toward the goal over
    # `stuck_window_s` while the controller was actively trying, we
    # abort the goal so the upstream planner (explore) picks a new one.
    last_pose_at: float = 0.0
    last_pose_xy: Optional[Tuple[float, float]] = None
    last_dist_to_goal: float = float("inf")
    # Lethal-halo bounce detector. When the robot's center walks into
    # an inscribed-obstacle cell the tick reverses (-0.10, 0). Next
    # tick the new pose is usually outside the halo, the planner picks
    # the same target, the follower drives forward — and back into the
    # same cell. Without bounce-counting the robot wiggles in place
    # forever (or until the 8s stuck detector fires, but at 0.10 m/s
    # reverse + replan you can sustain enough net motion to dodge that).
    # We count consecutive lethal-halo entries within a short window;
    # >= 3 in `lethal_bounce_window_s` aborts the goal so explore picks
    # a different frontier instead of inflating the same halo.
    lethal_bounce_count: int = 0
    lethal_bounce_first_at: float = 0.0


class NavNode:
    """Lifecycle: construct → start() spawns rclpy node + spin thread.
    `set_goal(...)` swaps the active goal atomically and triggers
    a replan; `cancel_goal()` aborts."""

    def __init__(self, *, scan_topic: str = "/scanner",
                  odom_topic: str = "/odom",
                  cmd_topic: str = "/cmd_vel",
                  map_topic: str = "/map",
                  pose_topic: Optional[str] = None,
                  depth_topic: Optional[str] = None,
                  goal_topic: str = "/goal_pose",
                  path_pub_topic: str = "/simple_nav/path") -> None:
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.cmd_topic = cmd_topic
        self.map_topic = map_topic
        self.pose_topic = pose_topic
        # Depth image (sensor_msgs/Image, 32FC1 metres or 16UC1 mm) used
        # for the second-line forward e-stop. 2D lidar at chassis height
        # passes through tall thin obstacles (potted plants, table legs
        # under thin tablecloths, dangling wires) — depth catches them.
        # Optional: deployments without an RGBD camera fall back to
        # lidar-only e-stop.
        self.depth_topic = depth_topic
        self.goal_topic = goal_topic
        self.path_pub_topic = path_pub_topic

        self._lock = threading.Lock()
        self._ros = None
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        self._cmd_pub = None
        self._path_pub = None
        self._costmap_pub = None
        self._tf_buf = None
        self._tf_listener = None

        # Frames. base_link is the robot, map is the SLAM frame, odom is
        # the chassis driver's continuous frame. We plan in map.
        self._base_frame = "base_link"
        self._map_frame = "map"
        self._odom_frame = "odom"

        # State (map-frame robot pose, computed via tf in _on_odom or tick).
        self._pose: Optional[Tuple[float, float, float]] = None
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._scan = None
        self._depth: Optional[Any] = None  # latest sensor_msgs/Image
        self._map: Optional[Any] = None  # latest OccupancyGrid
        # Bumped on every /map callback; the active goal stores the
        # value at plan time so the tick can detect "map updated since
        # we planned" and re-validate the cached path. See Goal.plan_map_counter.
        self._map_counter: int = 0
        self._goal: Optional[Goal] = None
        self._last_v = 0.0  # for velocity-scaled lookahead

        # Tunables (could come from a yaml later).
        self._rpp_params = _follower.RPPParams()

    # ── lifecycle ───────────────────────────────────────────────────
    def start(self) -> None:
        if self._ros is not None:
            return
        self._ros = _import_ros()
        rclpy = self._ros["rclpy"]
        rclpy.init(args=None)
        Node = self._ros["Node"]
        node = Node("simple_nav")
        self._node = node

        QoS = self._ros["QoSProfile"]
        Rel = self._ros["ReliabilityPolicy"]
        Dur = self._ros["DurabilityPolicy"]
        Hist = self._ros["HistoryPolicy"]

        odom_qos = QoS(reliability=Rel.RELIABLE, durability=Dur.VOLATILE,
                        history=Hist.KEEP_LAST, depth=10)
        scan_qos = QoS(reliability=Rel.RELIABLE, durability=Dur.VOLATILE,
                        history=Hist.KEEP_LAST, depth=2)
        # /map is published TRANSIENT_LOCAL by both slam_toolbox and
        # cartographer's occupancy_grid_node; subscriber must match.
        map_qos = QoS(reliability=Rel.RELIABLE, durability=Dur.TRANSIENT_LOCAL,
                       history=Hist.KEEP_LAST, depth=1)

        node.create_subscription(self._ros["Odometry"], self.odom_topic,
                                  self._on_odom, odom_qos)
        node.create_subscription(self._ros["LaserScan"], self.scan_topic,
                                  self._on_scan, scan_qos)
        node.create_subscription(self._ros["OccupancyGrid"], self.map_topic,
                                  self._on_map, map_qos)
        if self.pose_topic:
            node.create_subscription(self._ros["PoseWithCovarianceStamped"],
                                      self.pose_topic, self._on_pose, odom_qos)
        if self.depth_topic:
            # Cameras spam at >10 Hz; we only need the freshest frame
            # for the per-tick e-stop check. KEEP_LAST(1) is enough.
            depth_qos = QoS(reliability=Rel.RELIABLE, durability=Dur.VOLATILE,
                            history=Hist.KEEP_LAST, depth=1)
            node.create_subscription(self._ros["Image"], self.depth_topic,
                                      self._on_depth, depth_qos)
        if self.goal_topic:
            node.create_subscription(self._ros["PoseStamped"],
                                      self.goal_topic, self._on_goal_pose, odom_qos)

        self._cmd_pub = node.create_publisher(self._ros["Twist"],
                                                self.cmd_topic, 10)
        self._path_pub = node.create_publisher(self._ros["Path"],
                                                 self.path_pub_topic, 1)
        # Republish the costmap as an OccupancyGrid on a dedicated topic
        # so rviz can render it as a heatmap layered on /map. Encoding:
        #   -1   unknown (matches /map convention)
        #   0    free (no obstacle nearby)
        #   1-98 inflated (gradient cost; higher = closer to obstacle)
        #   99   lethal (robot center forbidden — inscribed halo)
        # TRANSIENT_LOCAL so rviz can subscribe late and still get the
        # latest published costmap without us re-publishing on a timer.
        costmap_qos = QoS(reliability=Rel.RELIABLE, durability=Dur.TRANSIENT_LOCAL,
                          history=Hist.KEEP_LAST, depth=1)
        self._costmap_pub = node.create_publisher(self._ros["OccupancyGrid"],
                                                    "/simple_nav/costmap",
                                                    costmap_qos)

        # tf2: SLAM (cartographer / slam_toolbox) publishes map → odom.
        # The chassis primitive / diffdrive controller publishes odom →
        # base_link. We need base_link in map frame for planning, so
        # walk the chain via Buffer.
        self._tf_buf = self._ros["Buffer"]()
        self._tf_listener = self._ros["TransformListener"](self._tf_buf, node)

        # 10 Hz control tick.
        node.create_timer(0.1, self._tick)

        self._spin_thread = threading.Thread(target=self._spin,
                                               name="simple-nav-spin",
                                               daemon=True)
        self._spin_thread.start()
        print(
            f"[simple_nav] node started: subs odom={self.odom_topic} "
            f"scan={self.scan_topic} map={self.map_topic} "
            f"pose={self.pose_topic or '(none)'} "
            f"depth={self.depth_topic or '(none)'} goal={self.goal_topic} "
            f"pub cmd={self.cmd_topic} path={self.path_pub_topic} "
            f"tick=10Hz",
            flush=True,
        )

    def stop(self) -> None:
        if self._ros is None:
            return
        try:
            self._publish_twist(0.0, 0.0)
        except Exception:
            pass
        self._stop_evt.set()
        try: self._node.destroy_node()
        except Exception: pass
        try: self._ros["rclpy"].shutdown()
        except Exception: pass
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)

    def _spin(self) -> None:
        # Use rclpy.spin(node) — the blocking variant — instead of
        # spin_once-in-loop. spin_once silently swallowing exceptions
        # was causing callbacks to stop firing when one threw (timer
        # tick collisions, tf lookup failures, etc.). spin() handles
        # that internally and never silently bails.
        rclpy = self._ros["rclpy"]
        try:
            rclpy.spin(self._node)
        except Exception as e:
            print(f"[simple_nav] spin exited: {e}", flush=True)

    # ── Sub callbacks ───────────────────────────────────────────────
    def _on_odom(self, msg) -> None:
        p = msg.pose.pose
        x, y = p.position.x, p.position.y
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._odom_pose = (x, y, yaw)

    def _on_pose(self, msg) -> None:
        # PoseWithCovarianceStamped — same parsing path.
        self._on_odom(msg)

    def _refresh_map_pose(self) -> None:
        """Look up base_link in map frame via tf. Falls back to
        odom-frame pose when the map → odom transform isn't available
        yet (e.g. cartographer just started). Stored under self._pose
        for the control loop."""
        if self._tf_buf is None:
            with self._lock:
                if self._odom_pose is not None:
                    self._pose = self._odom_pose
            return
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_frame, self._base_frame,
                self._ros["Time"](),
                timeout=self._ros["Duration"](seconds=0.1),
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self._pose = (tx, ty, yaw)
        except Exception:
            with self._lock:
                if self._odom_pose is not None and self._pose is None:
                    self._pose = self._odom_pose

    def _on_scan(self, msg) -> None:
        with self._lock:
            self._scan = msg

    def _on_depth(self, msg) -> None:
        with self._lock:
            self._depth = msg

    def _on_map(self, msg) -> None:
        with self._lock:
            self._map = msg
            self._map_counter += 1
        # Publish a fresh costmap whenever the source map updates, so
        # rviz can show inflation/lethal layers even before a goal is
        # set. Computing the costmap requires running the same
        # build_costmap() the planner uses, but with no path needed.
        try:
            gm = _planner.GridMap.from_msg(msg)
            passable, cost = _planner.build_costmap(
                gm, inscribed_m=0.25, inflation_m=0.20)
            preview = _planner.PlanResult(path=[], gm=gm,
                                          passable=passable, cost=cost)
            self._publish_costmap(preview)
        except Exception:
            pass

    def _on_goal_pose(self, msg) -> None:
        import uuid
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        # Quaternion → yaw (z rotation only — Force3DoF world).
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        gyaw = math.atan2(2.0 * (qw * qz + qx * qy),
                           1.0 - 2.0 * (qy * qy + qz * qz))
        print(f"[simple_nav] /goal_pose received: ({gx:.2f}, {gy:.2f}, yaw={math.degrees(gyaw):.1f}°) "
              f"frame={msg.header.frame_id}", flush=True)
        self.set_goal(Goal(
            goal_id="rviz-" + uuid.uuid4().hex[:6],
            target_x=gx, target_y=gy,
            target_yaw=gyaw,
            tolerance_m=self._rpp_params.goal_tolerance_m,
        ))

    # ── Goal API ────────────────────────────────────────────────────
    def set_goal(self, goal: Goal) -> None:
        with self._lock:
            if self._goal is not None and self._goal.state == "active":
                self._goal.state = "cancelled"
                self._goal.detail = "preempted by new goal"
            # New goal — clear cached path; will replan on the next tick.
            goal.path = None
            self._goal = goal

    def cancel_goal(self, goal_id: Optional[str] = None) -> bool:
        with self._lock:
            if self._goal is None:
                return False
            if goal_id is not None and self._goal.goal_id != goal_id:
                return False
            if self._goal.state == "active":
                self._goal.state = "cancelled"
                self._goal.detail = "cancelled by client"
                self._publish_path([])
                return True
            return False

    def goal_status(self, goal_id: Optional[str] = None) -> Optional[dict]:
        with self._lock:
            g = self._goal
            if g is None:
                return None
            if goal_id is not None and g.goal_id != goal_id:
                return None
            x, y, yaw = self._pose if self._pose else (None, None, None)
            return {
                "goal_id": g.goal_id,
                "target_x": g.target_x,
                "target_y": g.target_y,
                "state": g.state,
                "detail": g.detail,
                "elapsed_s": time.time() - g.started_at,
                "path_len": len(g.path) if g.path else 0,
                "robot_x": x, "robot_y": y, "robot_yaw": yaw,
            }

    # ── Control loop ────────────────────────────────────────────────
    def _tick(self) -> None:
        # Always refresh map-frame pose first via tf — odom-frame is
        # not enough since cartographer/slam_toolbox publishes a
        # non-identity map → odom transform we MUST honour.
        self._refresh_map_pose()
        with self._lock:
            g = self._goal
            pose = self._pose
            scan = self._scan
            mp = self._map
            depth = self._depth

        # Idle heartbeat — every ~10s, prove the timer is firing even
        # with no goal. Without this, "I sent /goal_pose but the node
        # looked dead" is indistinguishable from "the timer never
        # registered" (e.g. on_init returned early).
        now_t = time.time()
        if not hasattr(self, "_last_heartbeat") or now_t - self._last_heartbeat >= 10.0:
            have = {
                "pose": pose is not None,
                "scan": scan is not None,
                "map":  mp is not None,
                "goal": g is not None,
            }
            print(f"[simple_nav] heartbeat: {have}", flush=True)
            self._last_heartbeat = now_t

        if g is None or g.state != "active":
            # Surface the goal's terminal state once — without this,
            # an aborted goal looks identical to "no goal" in the
            # heartbeat output.
            if g is not None and not getattr(g, "_terminal_logged", False):
                print(
                    f"[simple_nav] goal {g.goal_id} -> {g.state}: {g.detail}",
                    flush=True,
                )
                g._terminal_logged = True
            return
        # Diagnostic: when there's an active goal but a missing input,
        # log the gap once per 2s so the operator sees why nothing moves.
        # (Silently dropping the tick is the source of "goal received,
        # then nothing happens" tickets.)
        now_t = time.time()
        if not hasattr(self, "_last_input_log") or now_t - self._last_input_log >= 2.0:
            missing = [
                n for (n, v) in (
                    ("pose", pose), ("scan", scan), ("map", mp),
                ) if v is None
            ]
            if missing:
                print(
                    f"[simple_nav] tick: active goal {g.goal_id} but "
                    f"missing inputs {missing} — waiting",
                    flush=True,
                )
                self._last_input_log = now_t
        if pose is None:
            return

        # Per-2s "what is the goal doing" log so the operator sees
        # phase/plan attempts/distance/detail in real time, not just
        # `goal=True` from the heartbeat.
        if not hasattr(self, "_last_goal_log") or now_t - self._last_goal_log >= 2.0:
            rx, ry, ryaw = pose
            dist = math.hypot(g.target_x - rx, g.target_y - ry)
            print(
                f"[simple_nav] goal {g.goal_id} phase={g.phase} "
                f"state={g.state} dist={dist:.2f}m "
                f"path_len={len(g.path) if g.path else 0} "
                f"plan_attempts={g.plan_attempts} "
                f"detail={g.detail!r}",
                flush=True,
            )
            self._last_goal_log = now_t

        # ── Hard emergency stop (360° proximity) ──────────────────────
        # Independent of the planner / costmap / pose tracking. Reads
        # the raw 2D lidar scan and aborts the goal if ANY beam (full
        # 360°, not just front arc) is closer than EMERGENCY_STOP_M.
        # Rationale: when the robot is wedged between a chair leg and
        # a table leg, the front beam may be clear (it's pointed at the
        # gap) while a side beam is 8 cm from the chair. Forward-only
        # check would let the controller keep "rotating to align" and
        # bashing the side. 360° says "anything around me is too
        # close → abort, let upstream pick another goal."
        # Tuned for Tiago (50 cm wide footprint): 0.25 m halts before
        # the body touches a wall.
        EMERGENCY_STOP_M = 0.25
        nearest_beam = _min_range_360(scan)
        if nearest_beam is not None and nearest_beam < EMERGENCY_STOP_M:
            self._publish_twist(0.0, 0.0)
            self._publish_path([])
            g.state = "aborted"
            g.detail = (f"emergency stop: lidar {nearest_beam:.2f}m "
                        f"(threshold {EMERGENCY_STOP_M:.2f}m, 360°)")
            return

        # ── Depth-based forward e-stop (second line) ─────────────────
        # 2D lidar at chassis height misses tall thin obstacles —
        # potted plants whose stem is above the lidar plane, table
        # legs blocked by a low fringe, etc. Webots tiago demo: the
        # robot drove straight into a potted plant because its lidar
        # rays passed between the leaves. Depth from the head camera
        # catches anything in the forward cone that has volume above
        # chassis height.
        #
        # Threshold tighter than lidar (0.30 m vs 0.25 m): depth has
        # noisy edge pixels at long range that we don't want to abort
        # on at the same distance the lidar would. Aborts the goal
        # like the lidar e-stop does — explore picks a new frontier,
        # nav doesn't try to inch around the obstacle.
        DEPTH_STOP_M = 0.30
        nearest_depth = _forward_depth_min(depth)
        if nearest_depth is not None and nearest_depth < DEPTH_STOP_M:
            self._publish_twist(0.0, 0.0)
            self._publish_path([])
            g.state = "aborted"
            g.detail = (f"emergency stop: depth {nearest_depth:.2f}m "
                        f"(threshold {DEPTH_STOP_M:.2f}m, forward cone)")
            return

        # ── Stuck detector ─────────────────────────────────────────────
        # Snapshot pose every tick. After STUCK_WINDOW_S of trying, if
        # we've moved less than STUCK_PROGRESS_M and we're not within
        # arrival tolerance, abort the goal. The upstream planner
        # (explore) will pick a new frontier; without this, the robot
        # oscillates inside the inflation halo of e.g. a chair and
        # bombards /cmd_vel with conflicting commands ("twitching").
        STUCK_WINDOW_S = 8.0
        STUCK_PROGRESS_M = 0.15
        now = time.time()
        rx, ry, _ = pose
        dist_to_goal = math.hypot(g.target_x - rx, g.target_y - ry)
        if g.last_pose_at == 0.0:
            g.last_pose_at = now
            g.last_pose_xy = (rx, ry)
            g.last_dist_to_goal = dist_to_goal
        elif now - g.last_pose_at >= STUCK_WINDOW_S:
            moved = math.hypot(rx - g.last_pose_xy[0], ry - g.last_pose_xy[1])
            progress = g.last_dist_to_goal - dist_to_goal
            # Both raw motion AND progress-toward-goal must be tiny —
            # robot wiggling in place still has small motion that we
            # don't want to count as "alive".
            if moved < STUCK_PROGRESS_M and progress < STUCK_PROGRESS_M and dist_to_goal > g.tolerance_m + 0.2:
                g.state = "aborted"
                g.detail = (f"stuck: moved {moved:.2f}m, progress {progress:+.2f}m "
                            f"toward goal in {STUCK_WINDOW_S:.0f}s")
                self._publish_twist(0.0, 0.0)
                self._publish_path([])
                return
            g.last_pose_at = now
            g.last_pose_xy = (rx, ry)
            g.last_dist_to_goal = dist_to_goal

        # ── Map-driven replan ───────────────────────────────────────────
        # If the map has updated since we planned, re-validate the
        # cached path against the fresh costmap. A path waypoint that
        # was free at plan time may now sit inside a wall the SLAM
        # service just discovered (the robot's lidar swept a previously-
        # unknown corner, rtabmap committed it to /map). Walking blindly
        # into the new obstacle is what made the user catch us — robot
        # bashed against a freshly-mapped table for 8s before the stuck
        # detector aborted the goal.
        with self._lock:
            current_map_counter = self._map_counter
        if (g.path is not None and mp is not None
                and g.plan_map_counter < current_map_counter):
            try:
                gm = _planner.GridMap.from_msg(mp)
                passable, cost = _planner.build_costmap(
                    gm, inscribed_m=0.35, inflation_m=0.40)
                # Sample the path at every waypoint past the robot's
                # current position. Cells outside the grid are treated
                # as passable so a goal beyond the current map edge
                # isn't punished (matches `_passable_at` semantics).
                blocked = False
                for (wx, wy) in g.path:
                    cx, cy = gm.world_to_cell(wx, wy)
                    if not gm.in_bounds(cx, cy):
                        continue
                    if not bool(passable[cy, cx]):
                        blocked = True
                        break
                if blocked:
                    g.path = None
                    g.detail = "replan: cached path blocked by updated map"
                    g.costmap = _planner.PlanResult(
                        path=[], gm=gm, passable=passable, cost=cost)
                    self._publish_path([])
                    g.plan_attempts = 0
                else:
                    # Path still valid — refresh costmap so the follower
                    # uses the latest cost layer (gradients near newly
                    # mapped walls slow us down even before they block).
                    g.costmap = _planner.PlanResult(
                        path=g.path, gm=gm, passable=passable, cost=cost)
            except Exception as e:  # noqa: BLE001
                g.detail = f"path re-validation failed: {e}"
            g.plan_map_counter = current_map_counter

        # ── Replan on demand: if path is None we just got a new goal,
        # or the previous plan failed, or the map-driven replan above
        # invalidated the cached path; try again with the current map.
        if g.path is None:
            if mp is None:
                # Map not yet available — cannot plan. Bail; we'll try
                # again next tick. Don't fail the goal, mapping might
                # still come up.
                g.plan_attempts += 1
                if g.plan_attempts == 1:
                    g.detail = "waiting for /map"
                return
            try:
                gm = _planner.GridMap.from_msg(mp)
                # Tiago footprint ≈ 50cm diameter → 25cm inscribed.
                # 20cm inflation tail biases A* toward corridor centers
                # so PP tracking error doesn't push the body into
                # walls. Total preferred clearance from obstacle to
                # robot center = 45cm.
                result = _planner.plan_world(
                    gm,
                    (pose[0], pose[1]),
                    (g.target_x, g.target_y),
                    # Tiago footprint ≈ 50 cm; Webots scenes have lots
                    # of chairs/tables clustered, so the planned path
                    # MUST stay further from obstacles than the lidar
                    # alone would suggest. 35 cm inscribed + 40 cm
                    # inflation tail keeps the body 75 cm minimum from
                    # any wall/leg, which gave us the "wedged between
                    # chairs" failure mode at 0.25/0.20.
                    inscribed_m=0.35,
                    inflation_m=0.40,
                )
                path = result.path
                g.costmap = result        # follower will sample cost
                self._publish_costmap(result)
            except Exception as e:  # noqa: BLE001
                path = None
                g.detail = f"plan exception: {e}"
                print(f"[simple_nav] plan exception: {e}", flush=True)
            if path is None or len(path) < 2:
                g.plan_attempts += 1
                if g.plan_attempts == 1 or g.plan_attempts % 10 == 0:
                    print(
                        f"[simple_nav] planner returned no path "
                        f"(attempt {g.plan_attempts}): start=({pose[0]:.2f},{pose[1]:.2f}) "
                        f"goal=({g.target_x:.2f},{g.target_y:.2f}) detail={g.detail!r}",
                        flush=True,
                    )
                if g.plan_attempts > 50:   # ~5s of failed planning
                    g.state = "aborted"
                    g.detail = "no path to goal (after 50 attempts)"
                    self._publish_twist(0.0, 0.0)
                return
            g.path = path
            g.plan_map_counter = current_map_counter
            g.detail = f"plan ok ({len(path)} pts)"
            print(
                f"[simple_nav] planned: {len(path)} pts, "
                f"({pose[0]:.2f},{pose[1]:.2f}) -> "
                f"({g.target_x:.2f},{g.target_y:.2f})",
                flush=True,
            )
            self._publish_path(path)

        # ── Run RPP follower against the cached path. ────────────────
        forward_clear = _forward_clearance(scan)
        # Sample the costmap at the robot's current pose: lethal-zone
        # entry triggers an immediate back-off (collision-imminent
        # without waiting for the lidar regulator), and the gradient
        # cost slows the follower when we're drifting close to a wall.
        cm = getattr(g, "costmap", None)
        cost_at_robot = _costmap_at(cm, pose[0], pose[1]) if cm is not None else 0.0
        in_lethal = (cm is not None and not _passable_at(cm, pose[0], pose[1]))

        if in_lethal:
            if not hasattr(self, "_last_lethal_log") or now_t - self._last_lethal_log >= 2.0:
                print(
                    f"[simple_nav] in lethal halo at ({pose[0]:.2f},{pose[1]:.2f}) "
                    f"cost={cost_at_robot:.2f}; reversing",
                    flush=True,
                )
                self._last_lethal_log = now_t
            # Robot center inside an obstacle's inscribed halo.
            # Reverse straight back along current heading to escape;
            # don't try to plan from a forbidden cell.
            #
            # Bounce-detect: if we've come back here >= 3 times in 5 s
            # the planner is stuck oscillating around the inflation
            # boundary of an obstacle that's right on the path to the
            # goal. Abort and let explore pick a different frontier
            # instead of inflating the same halo into the next minute.
            now = time.time()
            LETHAL_BOUNCE_WINDOW_S = 5.0
            LETHAL_BOUNCE_LIMIT = 3
            if now - g.lethal_bounce_first_at > LETHAL_BOUNCE_WINDOW_S:
                g.lethal_bounce_count = 1
                g.lethal_bounce_first_at = now
            else:
                g.lethal_bounce_count += 1
            if g.lethal_bounce_count >= LETHAL_BOUNCE_LIMIT:
                g.state = "aborted"
                g.detail = (f"oscillating at lethal halo "
                            f"({g.lethal_bounce_count} entries in "
                            f"{now - g.lethal_bounce_first_at:.1f}s)")
                self._publish_twist(0.0, 0.0)
                self._publish_path([])
                return
            self._publish_twist(-0.10, 0.0)
            g.detail = (f"in lethal halo; reversing "
                        f"(bounce {g.lethal_bounce_count}/{LETHAL_BOUNCE_LIMIT})")
            # Force replan once we're out.
            g.path = None
            g.costmap = None
            return

        # ── Align phase: xy reached, now rotate to target_yaw. ──────
        # PP naturally arrives pointing along the last path segment,
        # which generally won't match the user's requested yaw. Once
        # we're close enough in xy, switch to in-place rotation until
        # heading matches.
        if g.phase == "align":
            if g.target_yaw is None:
                g.state = "succeeded"
                g.detail = "goal reached (no yaw target)"
                self._publish_twist(0.0, 0.0)
                self._publish_path([])
                return
            yaw_err = g.target_yaw - pose[2]
            while yaw_err > math.pi:  yaw_err -= 2 * math.pi
            while yaw_err < -math.pi: yaw_err += 2 * math.pi
            if abs(yaw_err) < g.yaw_tolerance_rad:
                g.state = "succeeded"
                g.detail = f"goal reached (yaw err {math.degrees(yaw_err):.1f}°)"
                self._publish_twist(0.0, 0.0)
                self._publish_path([])
                return
            # Pure rotation, sign of error, capped magnitude. Slower
            # near the target so we don't overshoot.
            vw = max(-0.6, min(0.6, 1.5 * yaw_err))
            self._publish_twist(0.0, vw)
            g.detail = f"aligning yaw (err {math.degrees(yaw_err):.1f}°)"
            return

        v, w, mode = _follower.compute(
            self._rpp_params, g.path, pose,
            forward_clearance_m=forward_clear,
            current_linear_vel=self._last_v,
        )
        if not hasattr(self, "_last_follower_log") or now_t - self._last_follower_log >= 2.0:
            print(
                f"[simple_nav] follower: v={v:.2f} w={w:.2f} mode={mode} "
                f"clearance={forward_clear:.2f}m cost_at_robot={cost_at_robot:.2f}",
                flush=True,
            )
            self._last_follower_log = now_t
        # Costmap-aware speed cap: scale v by (1 - cost_at_robot). A
        # robot center at the lethal boundary (cost=1) crawls; in the
        # middle of an open room (cost≈0) full desired speed.
        v = v * max(0.0, 1.0 - 0.85 * cost_at_robot)
        self._last_v = v
        if mode == "arrived":
            # xy reached. If the goal carries a target_yaw, switch to
            # the align phase (next tick handles in-place rotation);
            # otherwise we're done immediately.
            if g.target_yaw is None:
                g.state = "succeeded"
                g.detail = "goal reached"
                self._publish_twist(0.0, 0.0)
                self._publish_path([])
                return
            g.phase = "align"
            self._publish_twist(0.0, 0.0)
            self._publish_path([])
            g.detail = "xy reached; aligning yaw"
            return
        if mode == "blocked":
            # Forward path obstructed within ~30cm. Rather than just
            # stopping & re-planning the same straight line, rotate
            # in place toward the goal so we open a new heading. RPP
            # will pick a different lookahead next tick when the
            # arc is no longer pointing at a wall. Cap with a small
            # angular vel so we don't spin wildly.
            rx, ry, ryaw = pose
            dx, dy = g.target_x - rx, g.target_y - ry
            target_yaw = math.atan2(dy, dx)
            yaw_err = target_yaw - ryaw
            while yaw_err > math.pi: yaw_err -= 2 * math.pi
            while yaw_err < -math.pi: yaw_err += 2 * math.pi
            v_ang = max(-0.6, min(0.6, 0.8 * yaw_err if abs(yaw_err) > 0.1 else 0.4))
            self._publish_twist(0.0, v_ang)
            g.detail = "blocked; rotating"
            return
        self._publish_twist(v, w)

    # ── publishing helpers ──────────────────────────────────────────
    def _publish_twist(self, linear: float, angular: float) -> None:
        if self._cmd_pub is None:
            return
        Twist = self._ros["Twist"]
        m = Twist()
        m.linear.x = float(linear)
        m.angular.z = float(angular)
        self._cmd_pub.publish(m)

    def _publish_path(self, path: List[Tuple[float, float]]) -> None:
        if self._path_pub is None:
            return
        Path = self._ros["Path"]
        PoseStamped = self._ros["PoseStamped"]
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        try:
            self._path_pub.publish(msg)
        except Exception:
            pass

    def _publish_costmap(self, plan_result) -> None:
        """Publish the planner's costmap as an OccupancyGrid for rviz.

        Encoding (matches nav2's `costmap_2d` convention enough for the
        Map display to colour-grade it usefully):

        * unknown cells in source map → -1 (rviz draws them grey)
        * passable cells with cost ≈ 0 → 0  (free, dark blue)
        * passable cells in inflation halo → 1..98 (gradient, scaled
          from the planner's [0,1] cost field)
        * inscribed/lethal cells → 99 (bright red in rviz)

        Frame, resolution, origin all copied from the source GridMap so
        rviz overlays the costmap exactly on top of /map.
        """
        if self._costmap_pub is None or plan_result is None:
            return
        OccupancyGrid = self._ros["OccupancyGrid"]
        gm = plan_result.gm
        passable = plan_result.passable
        cost = plan_result.cost
        h, w = gm.height, gm.width

        import numpy as np
        out = np.full((h, w), -1, dtype=np.int8)
        # Source map's known/unknown is reused for "unknown" cells: a
        # cell beyond the SLAM frontier should still appear as unknown,
        # not as costmap-free.
        src = gm.data
        known = src >= 0
        # Free baseline = where source is known.
        out[known] = 0
        # Inflation gradient (cost in [0,1] → 1..98 occupancy).
        # Mask: passable AND in the inflation halo (cost > 0).
        infl = passable & (cost > 0.0)
        out[infl] = np.clip((cost[infl] * 98.0).astype(np.int32) + 1, 1, 98).astype(np.int8)
        # Lethal: not passable in known space.
        lethal = (~passable) & known
        out[lethal] = 99

        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = float(gm.resolution)
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = float(gm.origin_x)
        msg.info.origin.position.y = float(gm.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = out.flatten().tolist()
        try:
            self._costmap_pub.publish(msg)
        except Exception:
            pass


# ── Helpers ────────────────────────────────────────────────────────
def _forward_clearance(scan) -> Optional[float]:
    """Min range in the front ±30° arc."""
    if scan is None:
        return None
    n = len(scan.ranges)
    if n == 0:
        return None
    mid = n // 2
    half = max(1, n // 6)
    lo, hi = max(0, mid - half), min(n, mid + half)
    arc = [r for r in scan.ranges[lo:hi] if math.isfinite(r) and r > 0.05]
    if not arc:
        return None
    return min(arc)


def _forward_depth_min(depth_msg) -> Optional[float]:
    """Sample the central horizontal strip of a depth image and return
    the smallest valid (non-zero, finite, non-floor) distance in metres,
    or None if no usable depth samples are available.

    The strip is the middle vertical third of the image (skip ceiling
    + floor) and the central horizontal third (forward cone, roughly
    matches the lidar's ±30° forward arc). We're looking for "is
    there a solid object less than ~0.3 m in front of the robot",
    not building a full free-space map — sub-sampling is fine and
    keeps the per-tick cost trivial.

    Encoding handling: 32FC1 metres (webots, RealSense), 16UC1 mm
    (Astra, Kinect). Anything else returns None and the caller falls
    back to lidar-only e-stop.

    Filters out: NaN/Inf (no return), 0 (no return on int encodings),
    < 0.05 m (sensor self-floor / cable reflections).
    """
    if depth_msg is None:
        return None
    h, w = int(getattr(depth_msg, "height", 0)), int(getattr(depth_msg, "width", 0))
    if h == 0 or w == 0:
        return None
    enc = (getattr(depth_msg, "encoding", "") or "").lower()
    try:
        import numpy as np
        if enc in ("32fc1", "32fc1; 32fc1"):
            arr = np.frombuffer(bytes(depth_msg.data), dtype=np.float32).reshape(h, w)
        elif enc in ("16uc1", "16uc1; 16uc1"):
            arr = np.frombuffer(bytes(depth_msg.data), dtype=np.uint16).reshape(h, w).astype(np.float32) / 1000.0
        else:
            return None
        # Central vertical third × central horizontal third.
        y0, y1 = h // 3, (2 * h) // 3
        x0, x1 = w // 3, (2 * w) // 3
        patch = arr[y0:y1, x0:x1]
        valid = np.isfinite(patch) & (patch > 0.05)
        if not valid.any():
            return None
        return float(patch[valid].min())
    except Exception:
        return None


def _min_range_360(scan) -> Optional[float]:
    """Smallest valid range across the full scan (no arc filter).
    Used for the emergency-stop guard: any obstacle closer than
    EMERGENCY_STOP_M *anywhere* around the robot triggers an abort.
    Filters non-finite values and the typical lidar-self-return floor
    at 5 cm so the body doesn't trip on its own chassis reflections."""
    if scan is None:
        return None
    arc = [r for r in scan.ranges if math.isfinite(r) and r > 0.05]
    if not arc:
        return None
    return min(arc)


def _costmap_at(plan_result, x: float, y: float) -> float:
    """Sample the cost layer at world (x,y). Returns 0 outside the
    grid — caller treats that as "open space"."""
    if plan_result is None:
        return 0.0
    gm = plan_result.gm
    cx, cy = gm.world_to_cell(x, y)
    if not gm.in_bounds(cx, cy):
        return 0.0
    return float(plan_result.cost[cy, cx])


def _passable_at(plan_result, x: float, y: float) -> bool:
    """True iff world (x,y) is outside the lethal layer. Out-of-grid
    cells are treated as passable so a robot exploring past the map
    edge isn't punished."""
    if plan_result is None:
        return True
    gm = plan_result.gm
    cx, cy = gm.world_to_cell(x, y)
    if not gm.in_bounds(cx, cy):
        return True
    return bool(plan_result.passable[cy, cx])
