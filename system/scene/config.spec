# Runtime configuration accepted by the Scene service.
#
# This documents the mapping passed as the service instance's `config:` value
# in a deployment manifest, delivered through Driver(CMD_INIT) by `rbnx boot`.
# It is not loaded as a schema. Environment variables provide fallbacks for
# standalone use; instance config takes priority over them.
#
# Unknown keys under `perception.dualmap` fail Driver(CMD_INIT) naming the key
# and listing what is accepted. Unknown keys elsewhere are logged as ignored
# and the service starts.

config:

  # ── what the robot is looking through ────────────────────────────────────

  # list of mappings, no default. Which ROS 2 topics the ingest layer
  # subscribes to. Each entry pairs a logical kind with a topic and a message
  # class; `kind` is one of rgb, depth, lidar2d, pose, odom. Without an rgb
  # and a depth entry the perception loop has nothing to read and says so
  # rather than starting empty.
  observations:
    - kind: rgb
      topic: /head_front_camera/rgb/image_raw
      msg: sensor_msgs/msg/Image

  # string, default: ros2. Transport the observations arrive over.
  transport: ros2

  # string TF frame, default: "" (environment fallback SCENE_CAMERA_FRAME).
  # The frame the camera publishes in. Scene needs camera->map to place an
  # observation on the map; with neither this nor the fallback set it waits
  # rather than guessing a frame.
  camera_frame: ""

  # string TF frame, default: "" (environment fallback SCENE_BASE_FRAME).
  # The robot's own frame, used for the footprint and for reachability.
  base_frame: ""

  # optional string, default: "". Atlas provider id of the camera to read
  # intrinsics from when more than one is registered.
  camera_provider_id: ""

  # optional mapping, default: none. Pinhole intrinsics to use when the
  # camera publishes no CameraInfo. A deployment that needs this is usually
  # missing a driver; it exists so a bring-up is not blocked by one.
  intrinsics_fallback: null

  # finite float seconds, default: 2.0 (environment fallback
  # SCENE_POSE_MAX_AGE_S). A pose older than this is not used to place an
  # observation. A stale pose puts objects where the robot used to be.
  pose_max_age_s: 2.0

  # ── identity and interface ───────────────────────────────────────────────

  # optional string, default: none (environment fallback SCENE_MAP_ID).
  # The map this Scene instance binds to. Left unset, a boot starts a fresh
  # live session that the operator names when they save it -- a static
  # default would make a first run look like a loaded map.
  map_id: null

  # optional integer port, default: 50107 (environment fallback
  # SCENE_WEB_PORT). 0 disables the web interface.
  web_port: 50107

  # ── perception ───────────────────────────────────────────────────────────

  perception:

    # string, default: lite (environment fallback SCENE_PROFILE).
    # lite      the 2060-class model set, about 4 GB
    # full      the paper-tier set (SAM-L + CLIP ViT-H-14) from SCENE_MODELS_DIR
    # annotate  object recognition off; manual regions and geometric queries only
    profile: lite

    # string, default: concept_graphs (environment fallback
    # SCENE_PERCEPTION_BACKEND). Which open-vocabulary mapper runs on the
    # RGB-D stream: concept_graphs or dualmap. Both take the same inputs and
    # feed the same ObjectRegistry, so nothing above the detector changes.
    backend: concept_graphs

    # finite float seconds, default: backend-specific. How often the
    # perception loop runs. DualMap additionally gates on the keyframe rule
    # below, so a shorter period costs little there and costs a full
    # detect/segment/encode pass on concept_graphs.
    period_s: 1.0

    # finite float in [0, 1], default: backend-specific. Detections below
    # this score are dropped before association.
    confidence_threshold: 0.3

    # integer, default: backend-specific. Upper bound on detections carried
    # from one frame.
    max_detections: 80

    # mapping, default: {}. Passed to the ConceptGraphs backend. Accepted
    # keys are not validated here: an unrecognised one reaches the detector's
    # own defaults rather than failing the boot.
    concept_graphs: {}

    # mapping, default: {}. Passed to the DualMap backend. Every accepted key
    # is listed below; an unknown one fails Driver(CMD_INIT) naming it.
    #
    # The defaults are DualMap's own and are sized for a dataset replay that
    # maps every frame. A robot mapping keyframes at walking pace sees each
    # object a handful of times, so the four marked REQUIRED usually have to
    # be set for a deployment: left alone, one 180 s office run went from 65
    # tracks to 1 and the map emptied out behind the robot.
    dualmap:

      # REQUIRED. list of strings, default: DualMap's 101-name domestic list.
      # The YOLO-World vocabulary. A detector can only answer with a name it
      # was given: on an office world the domestic list cost 0.41 label
      # accuracy against 0.82 with a 40-name office list, calling the cabinet
      # a sink and the chair an ironing board.
      classes: [chair, table, sofa, door, window, lamp, monitor, shelf]

      # REQUIRED. integer, default: 8. Observations before a track counts as
      # stable. Match it to how often the robot actually sees a thing.
      stable_num: 3

      # integer, default: 10. How many recent frames count as active. A track
      # that leaves this window without becoming stable is dropped.
      active_window_size: 10

      # integer, default: 5. Rounds an unstable track survives outside that
      # window. Raising it also delays promotion to the global map.
      max_pending_count: 5

      # REQUIRED. finite float metres, default: 0.1. Map a frame only after
      # this much travel.
      keyframe_translation_m: 0.15

      # REQUIRED. finite float degrees, default: 3.0. Or this much turn.
      keyframe_rotation_deg: 5.0

      # finite float seconds, default: 5.0. Or this long since the last
      # mapped frame.
      keyframe_time_s: 5.0

      # finite float, default: 1.2. An observation joins an existing track
      # when cos(CLIP) plus point overlap exceeds this.
      sim_threshold: 1.2

      # finite float metres, default: 0.02. The radius "overlap" counts
      # within. It must exceed the SLAM pose error between keyframes, or two
      # views of one object never overlap and every keyframe starts a new
      # track.
      downsample_voxel_size: 0.05

      # integer keyframes, default: 20. Self-merge cadence for the local map;
      # 0 disables it.
      merge_every_keyframes: 20

      # finite float in [0, 1], default: 0.9. Point overlap two tracks need
      # before they are merged. Upstream's 0.9 never fires under a robot's
      # pose error.
      merge_sim_threshold: 0.3

      # boolean, default: false. Run DualMap's abstract (global) map as well.
      # It merges across classes by top-down 2D overlap, which is what
      # collapses one workstation reported as tv + speaker + desk -- but it
      # keeps only low-mobility anchors and drops every other stable track
      # once it leaves view. An inventory wants this off; a navigation memory
      # wants it on.
      global_map: false

      # integer, default: 1. Keyframes a track must be seen in before it
      # enters the ObjectRegistry.
      min_observations: 1

      # boolean, default: false. Additionally require DualMap's own stable
      # flag before a track enters the registry.
      stable_only: false

      # boolean, default: false. Keep FastSAM segments YOLO-World could not
      # name, as unlabelled objects.
      keep_unknown: false

      # boolean, default: follows keep_unknown. Run FastSAM at all; it only
      # contributes the unnamed segments keep_unknown admits.
      use_fastsam: false

      # boolean, default: false. Drop tracks whose points all lie within 5 cm
      # of the floor plane. Off by default because it also drops rugs and
      # carpets, which are real and thin.
      floor_gate: false

      # finite float metres, default: 0.0. Where that floor plane is.
      floor_z_m: 0.0

      # optional string torch device, default: cuda when available, else cpu.
      device: null
