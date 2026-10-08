# The mapping under `system.scene.config:` in robonix_manifest.yaml. rbnx boot
# delivers it to Scene through Driver(CMD_INIT). Keys written beside
# `manifest:` instead of under `config:` are still accepted, with a
# deprecation warning; under both, `config:` wins. RBNX_CONFIG_FILE is a
# deprecated fallback read only when CMD_INIT carries an empty mapping.
#
# CMD_INIT only stores the mapping and logs its keys. Values are checked when
# Scene starts its runtime on CMD_ACTIVATE, so a bad value fails activation,
# not init. An unknown key under perception.dualmap or perception.concept_graphs
# fails it, naming the key and listing the accepted ones. An unknown key
# directly under perception is logged as ignored. Unknown top-level keys are
# ignored.
#
# Environment variables are fallbacks for standalone use; a value here wins
# over them. The exception is the SCENE_CG_* variables named under
# perception.concept_graphs: they override the manifest.
#
#
# ─── Read this first: almost nothing here has to be set ──────────────────────
#
# Scene finds its inputs through Atlas. When a camera capability is registered,
# the auto-discovery loop resolves the RGB, depth, intrinsics and transform
# topics itself and logs what it bound to:
#
#     [scene] 'rgb' ← atlas: topic=/head_front_camera/rgb/image_raw msg=Image ...
#
# So the sensor keys (observations and the ones after it) are for the case
# where that does not work. A deployment that spells out inputs Atlas already
# knows about has two sources of truth and will eventually disagree with
# itself.
#
# What has to be set is four keys, all under perception.dualmap, and only when
# that backend runs:
#
#     perception.dualmap.classes                 the deployment's vocabulary
#     perception.dualmap.stable_num              observations to become stable
#     perception.dualmap.keyframe_translation_m  when a frame is mapped
#     perception.dualmap.keyframe_rotation_deg   likewise
#
# Their defaults come from DualMap upstream and are sized for a dataset replay
# that maps every frame. A robot mapping keyframes at walking pace sees each
# object a handful of times; at the upstream defaults one 180-second office run
# went from 65 tracks to 1, the map emptying out behind the robot. Scene does
# not enforce them, which is why they are not listed under `required`.
#
# Everything else has a working default. Properties are ordered accordingly:
# what to set, what to set occasionally, and what to leave alone unless
# something is wrong.

specVersion: 1
description: >-
  Configuration of Scene, the live semantic and geometric map of the robot's
  surroundings: which open-vocabulary mapper runs on the camera, the web
  interface, the map it binds to, and overrides for the sensor inputs that
  Atlas normally resolves.

properties:

  # ═══ 1. Set this ══════════════════════════════════════════════════════════

  perception:
    type: object
    default: {}
    x-group: Perception
    description: >-
      Object recognition. Must be a mapping. The tier follows the inputs: RGB
      and depth run the backend below, RGB alone runs a VLM detector with
      approximate positions, and no camera leaves only geometric queries.
    properties:

      backend:
        type: string
        enum: [concept_graphs, dualmap]
        description: >-
          Open-vocabulary mapper for the RGB-D stream. Both take the same
          inputs and feed the same ObjectRegistry, so nothing above the
          detector changes. Matched case-insensitively. When unset,
          SCENE_PERCEPTION_BACKEND applies, then dualmap if the image carries a
          DualMap checkout (SCENE_DUALMAP_ROOT, default /opt/dualmap, has a
          utils/ directory), else concept_graphs. An unknown name fails
          activation.
        # The backend also has to be known when the image is built: DualMap
        # brings its own checkout and weights. scripts/build.sh includes it on
        # x86-docker unless SCENE_PERCEPTION_BACKEND=concept_graphs. Starting
        # dualmap on an image without it logs "DualMap root /opt/dualmap has no
        # config/ directory" and fails activation with "perception.backend is
        # 'dualmap' but DualMap could not be loaded". That reads like a
        # configuration problem and is not one: build and boot with the same
        # backend.

      dualmap:
        type: object
        default: {}
        x-group: DualMap
        description: >-
          Settings of the DualMap backend; ignored by the other one. Must be a
          mapping, and an unknown key fails activation.
        properties:

          classes:
            type: array
            items:
              type: string
            description: >-
              Set this. The YOLO-World vocabulary. A detector can only answer
              with a name it was given: on an office world DualMap's domestic
              list scored 0.41 label accuracy against 0.82 for a 40-name
              office list, calling the cabinet a sink and the chair an ironing
              board. Wider than the room is fine; it keeps the score about
              recognition rather than lookup. Every item must be a string.
              When empty, the file named by SCENE_DUALMAP_CLASSES is used,
              else DualMap's config/class_list/gpt_indoor_general.txt.

          stable_num:
            type: integer
            description: >-
              Set this. Observations before a track counts as stable. Match it
              to how often the robot actually sees a thing. DualMap's default
              is 8, at which the map empties out behind a walking robot.

          keyframe_translation_m:
            type: number
            default: 0.1
            description: >-
              Set this. Metres of travel before the next frame is mapped.
              Feeding every frame fragments objects; a frame taken without
              motion adds nothing.

          keyframe_rotation_deg:
            type: number
            default: 3.0
            description: Set this. Degrees of turn before the next frame is mapped.

          keyframe_time_s:
            type: number
            default: 5.0
            description: Seconds after which a frame is mapped regardless of motion.

          sim_threshold:
            type: number
            description: >-
              CLIP cosine similarity plus point overlap a detection needs to
              join a track. DualMap's default is 1.2.

          downsample_voxel_size:
            type: number
            description: >-
              Metres; the radius point overlap is counted within. It must
              exceed the SLAM pose error between keyframes, or two views of
              one object never overlap and every keyframe starts a new track.
              DualMap's default is 0.02.

          merge_every_keyframes:
            type: integer
            default: 20
            description: Keyframes between merges of the local map; 0 disables merging.

          merge_sim_threshold:
            type: number
            description: >-
              Point overlap two tracks need to merge. DualMap's default is 0.9,
              which never fires under a robot's pose error.

          active_window_size:
            type: integer
            description: >-
              How many recent frames count as active. A track that leaves this
              window without becoming stable is dropped. DualMap's default is
              10.

          max_pending_count:
            type: integer
            description: >-
              Rounds an unstable track survives outside the active window.
              Raising it also delays promotion. DualMap's default is 5.

          min_observations:
            type: integer
            default: 1
            description: Detections a track needs before it enters the registry.

          stable_only:
            type: boolean
            default: false
            description: Also require DualMap's own stable flag before a track enters the registry.

          keep_unknown:
            type: boolean
            default: false
            description: Keep segments YOLO-World could not name, as `unknown`.

          use_fastsam:
            type: boolean
            description: >-
              Run FastSAM. Defaults to keep_unknown, since FastSAM only adds
              unnamed segments.

          global_map:
            type: boolean
            default: false
            description: >-
              Also run DualMap's abstract map. It merges across classes by
              top-down 2D overlap, the mechanism aimed at one workstation
              reported as tv + speaker + desk, but keeps only low-mobility
              anchors and drops every other stable track once it leaves view.
              An inventory wants this off; a navigation memory wants it on.

          floor_gate:
            type: boolean
            default: false
            description: >-
              Drop tracks whose 90th-percentile point height is less than 5 cm
              above the floor. Off because it also drops rugs.

          floor_z_m:
            type: number
            description: >-
              Height of the floor for floor_gate. Defaults to
              perception.concept_graphs.floor_z_m (0.0).

          device:
            type: [string, "null"]
            default: null
            description: >-
              Torch device. null means cuda when available, else cpu;
              SCENE_CG_FORCE_CPU forces cpu.

      # ═══ 2. Occasionally ══════════════════════════════════════════════════

      profile:
        type: string
        enum: [lite, full, annotate]
        default: lite
        description: >-
          lite is the small model set, about 4 GB of video memory. full is the
          paper-tier set (SAM-L and CLIP ViT-H-14) from SCENE_MODELS_DIR,
          mounted at /opt/models/full. annotate turns recognition off, leaving
          manual regions and geometric queries. Matched case-insensitively;
          an unknown name fails activation. Environment fallback:
          SCENE_PROFILE.

      period_s:
        type: number
        default: 0.6
        description: >-
          Seconds between perception passes. DualMap also gates on the
          keyframe rule, so a shorter period costs it little; on
          concept_graphs each pass is a full detect, segment and encode.
          Environment fallback: SCENE_DETECT_PERIOD_S. 0 means the default; a
          value that is not a number fails activation.

      confidence_threshold:
        type: number
        default: 0.3
        description: >-
          Detections scoring below this, in [0, 1], are dropped before
          association. Environment fallback: SCENE_DETECT_CONFIDENCE. 0 means
          the default; a value that is not a number fails activation.

      max_detections:
        type: integer
        default: 30
        description: >-
          Most detections carried from one frame. 0 means the default; a value
          that is not a number fails activation.

      concept_graphs:
        type: object
        default: {}
        x-group: ConceptGraphs
        description: >-
          Settings of the ConceptGraphs backend, thirty-nine of them. Must be a
          mapping, and an unknown key fails activation. Values are not
          type-checked when read. Where a SCENE_CG_* variable is named, it
          overrides the manifest value. The DualMap backend reads floor_z_m
          from here too.
        properties:

          # Merge and identity. Reach for these first: a run of office
          # shelving coming back as several objects means the association
          # gates are tighter than the pose error between views. These are
          # the knobs for the desk chair/table split and object dedup,
          # tunable on a running robot without a rebuild.

          merge_threshold:
            type: number
            default: 0.85
            x-group: Merge and identity
            description: >-
              Aggregated similarity a detection needs to be matched to an
              existing object. Overridden by SCENE_CG_MERGE_THRESHOLD.
          max_merge_dist_m:
            type: number
            default: 1.5
            x-group: Merge and identity
            description: >-
              Centroid distance gate on the per-tick merge. Overridden by
              SCENE_CG_MAX_MERGE_DIST_M.
          merge_overlap_thresh:
            type: number
            default: 0.5
            x-group: Merge and identity
            description: >-
              Point-overlap ratio the periodic merge pass needs.
              Overridden by SCENE_CG_MERGE_OVERLAP_THRESH.
          merge_visual_sim_thresh:
            type: number
            default: 0.65
            x-group: Merge and identity
            description: >-
              CLIP similarity the periodic merge pass needs once overlap
              passes. Overridden by SCENE_CG_MERGE_VISUAL_SIM_THRESH.
          merge_text_sim_thresh:
            type: number
            default: 0.0
            x-group: Merge and identity
            description: Text similarity the periodic merge pass needs.
          same_class_merge_dist_m:
            type: number
            default: 0.4
            x-group: Merge and identity
            description: >-
              Two objects of one class with centroids this close become one; 0
              disables. Overridden by SCENE_CG_SAME_CLASS_MERGE_DIST_M.
          same_class_merge_interval_ticks:
            type: integer
            default: 10
            x-group: Merge and identity
            description: Ticks between same-class merge passes.
          cross_class_centroid_max_m:
            type: number
            default: 0.5
            x-group: Merge and identity
            description: >-
              Centroid distance within which objects of different classes may
              merge. Overridden by SCENE_CG_CROSS_CLASS_CENTROID_MAX_M.
          cross_class_iou_thresh:
            type: number
            default: 0.3
            x-group: Merge and identity
            description: >-
              Box IoU a cross-class merge needs. Overridden by
              SCENE_CG_CROSS_CLASS_IOU_THRESH.
          cross_class_overlap_thresh:
            type: number
            default: 0.5
            x-group: Merge and identity
            description: >-
              Point overlap a cross-class merge needs. Overridden by
              SCENE_CG_CROSS_CLASS_OVERLAP_THRESH.
          cross_class_merge_interval_ticks:
            type: integer
            default: 10
            x-group: Merge and identity
            description: Ticks between cross-class merge passes.
          denoise_interval_ticks:
            type: integer
            default: 10
            x-group: Merge and identity
            description: Ticks between denoise passes.
          merge_overlap_interval_ticks:
            type: integer
            default: 10
            x-group: Merge and identity
            description: Ticks between periodic merge passes.

          # Association: how a detection is matched to an existing object.

          association:
            type: string
            default: voxel_vote
            x-group: Association
            description: >-
              voxel_vote scores a detection against each object by shared
              voxels and label similarity; any other value, such as cg, takes
              the older visual plus spatial similarity path. Overridden by
              SCENE_CG_ASSOCIATION.
          assoc_voxel_size_m:
            type: number
            default: 0.04
            x-group: Association
            description: >-
              Voxel size for voxel_vote, in metres. Must exceed the pose error
              between views. Overridden by SCENE_CG_ASSOC_VOXEL_SIZE_M.
          assoc_geo_weight:
            type: number
            default: 0.8
            x-group: Association
            description: Geometry's share of the voxel_vote score.
          assoc_feat_weight:
            type: number
            default: 0.2
            x-group: Association
            description: The label feature's share of the voxel_vote score.
          assoc_threshold:
            type: number
            default: 0.4
            x-group: Association
            description: >-
              Score a detection needs to join an object rather than start a new
              one. Overridden by SCENE_CG_ASSOC_THRESHOLD.
          spatial_sim_type:
            type: string
            default: iou
            x-group: Association
            description: >-
              iou or giou (axis-aligned boxes), iou_accurate or giou_accurate
              (oriented boxes), or overlap (voxel-grid point intersection,
              concept-graphs's canonical choice).
          match_method:
            type: string
            default: sim_sum
            x-group: Association
            description: How spatial and visual similarity are combined.
          phys_bias:
            type: number
            default: 0.0
            x-group: Association
            description: >-
              Above 0 trusts spatial similarity more than visual, below 0 the
              opposite.

          # Point clouds: what geometry an object is allowed to be made of.

          downsample_voxel_size:
            type: number
            default: 0.025
            x-group: Point clouds
            description: >-
              Open3D voxel size, in metres. Overridden by SCENE_CG_VOXEL_SIZE.
          min_points_threshold:
            type: integer
            default: 50
            x-group: Point clouds
            description: >-
              A detection with fewer points is dropped. Overridden by
              SCENE_CG_MIN_POINTS.
          obj_min_points:
            type: integer
            default: 20
            x-group: Point clouds
            description: >-
              An object with fewer points is dropped. Overridden by
              SCENE_CG_OBJ_MIN_POINTS.
          obj_pcd_max_points:
            type: integer
            default: 5000
            x-group: Point clouds
            description: >-
              Per-object point cap; the cloud is downsampled once it is
              passed. Overridden by SCENE_CG_OBJ_MAX_POINTS.
          obj_min_detections:
            type: integer
            default: 1
            x-group: Point clouds
            description: Detections before an object is kept.
          floor_z_m:
            type: number
            default: 0.0
            x-group: Point clouds
            description: >-
              Height of the floor in the world frame. Overridden by
              SCENE_CG_FLOOR_Z_M.
          dbscan_remove_noise:
            type: boolean
            default: true
            x-group: Point clouds
            description: Drop sparse outlier points with DBSCAN.
          dbscan_eps:
            type: number
            default: 0.1
            x-group: Point clouds
            description: DBSCAN cluster radius, in metres.
          dbscan_min_points:
            type: integer
            default: 10
            x-group: Point clouds
            description: DBSCAN minimum cluster size.
          per_detection_dbscan:
            type: boolean
            default: false
            x-group: Point clouds
            description: >-
              Denoise each detection before association, not only each object.
              Overridden by SCENE_CG_PER_DETECTION_DBSCAN.

          # Labels and features.

          label_vote:
            type: boolean
            default: true
            x-group: Labels and features
            description: >-
              Vote a label across observations rather than take the latest.
              Overridden by SCENE_CG_LABEL_VOTE.
          feature_bank_size:
            type: integer
            default: 32
            x-group: Labels and features
            description: CLIP features kept per object.
          feature_area_ratio:
            type: number
            default: 0.5
            x-group: Labels and features
            description: >-
              Share of an object's largest mask area a view needs for its
              feature to count. Overridden by SCENE_CG_FEATURE_AREA_RATIO.
          representative_by_text:
            type: boolean
            default: true
            x-group: Labels and features
            description: >-
              Pick the representative view by text similarity rather than by
              size. Overridden by SCENE_CG_REPRESENTATIVE_BY_TEXT.

          # Visibility: whether a miss means gone or means occluded.

          visibility_depth_margin_m:
            type: number
            default: 0.1
            x-group: Visibility
            description: >-
              The measured surface must be this far behind the object for a
              view to count as a miss.
          visibility_min_clear_samples:
            type: integer
            default: 3
            x-group: Visibility
            description: Clear depth samples a miss needs.
          visibility_min_clear_fraction:
            type: number
            default: 0.6
            x-group: Visibility
            description: Share of samples that must be clear for a miss.
          visibility_miss_ticks:
            type: integer
            default: 3
            x-group: Visibility
            description: Misses before an object is marked missing.

  # ═══ 2. Occasionally, continued ═══════════════════════════════════════════

  web_port:
    type: integer
    default: 50107
    x-group: Web interface
    description: >-
      Port of the web interface. 0 disables it. Environment fallback:
      SCENE_WEB_PORT.

  web_host:
    type: string
    default: 127.0.0.1
    x-group: Web interface
    description: >-
      Address the web interface binds to; 0.0.0.0 makes it reachable from
      other machines. Environment fallback: SCENE_WEB_HOST. Activation fails
      on a blank value, a URL, a path, whitespace, or a malformed IP address.

  web_viewer:
    type: string
    enum: [rerun, "off"]
    default: rerun
    x-group: Web interface
    description: >-
      rerun draws the 3D and 2D map pages; the scene docker image ships it,
      and when it is missing the pages say why. off never starts it. Matched
      case-insensitively; any other value fails activation. Environment
      fallback: SCENE_WEB_VIEWER. Ignored when web_port is 0.

  # These are environment variables only:
  #
  #   SCENE_RERUN_GRPC_PORT: 9876     the 3D map's gRPC port; the 2D map uses
  #                                   the next one. Local only: the browser
  #                                   reaches both through web_port.
  #   SCENE_RERUN_PERIOD_S: 1.0       how often the map pages are redrawn
  #   SCENE_RERUN_HISTORY: latest     latest keeps one value per entity, so the
  #                                   browser's memory stays bounded; changes
  #                                   keeps a timeline for debugging
  #   SCENE_RERUN_VIEWER_DIR: /opt/rerun-web-viewer
  #                                   where the image installed the viewer's
  #                                   browser bundle
  #   SCENE_OBJECT_VIEWS_DIR: unset   where each object's photographs are kept;
  #                                   unset keeps none, and then no object gets
  #                                   a VLM caption either (start.sh sets it)
  #   SCENE_OBJECT_VIEWS_MAX: 5       photographs kept per object
  #   SCENE_CAPTION_LANG: zh          language of VLM captions: zh or en
  #   SCENE_GRAPH_IMAGE_REFRESH_SEC: 600
  #                                   the same objects, unmoved, reuse the last
  #                                   image-relation answer for this long
  #                                   however the camera moves
  #   SCRIBE_LOG_DIR: set by rbnx     read by the logs page

  # ═══ 3. Leave alone unless something is wrong ═════════════════════════════
  #
  # These describe the sensor path, which Atlas normally resolves on its own.
  # Setting them is how a deployment overrides that discovery; it is not part
  # of a normal manifest.

  observations:
    type: array
    x-group: Sensor overrides
    description: >-
      Inputs to subscribe to, resolved through Atlas by contract. When given,
      only these are used: Scene neither waits for discovery nor adds inputs
      that appear later. Kinds Scene consumes are rgb, depth, lidar2d,
      lidar3d, camera_extrinsics, intrinsics, pose, odom, occupancy_grid and
      map_lifecycle. Without rgb and depth the metric mapper does not run.
    items:
      type: object
      required: [kind, contract]
      properties:
        kind:
          type: string
          description: Input kind, matched case-insensitively.
        contract:
          type: string
          description: Contract id, for example robonix/primitive/camera/rgb.
        msg_type:
          type: string
          description: >-
            ROS message type. Defaults to the one Scene knows for the contract;
            an entry with neither is skipped with a warning.
        provider_id:
          type: string
          x-provider: true
          default: ""
          description: >-
            Provider to read from. Empty means camera_provider_id for camera
            kinds and any provider otherwise.
    # An entry without kind or contract is skipped with a warning.

  transport:
    type: string
    enum: [ros2, grpc]
    default: ros2
    x-group: Sensor overrides
    description: >-
      Transport of the observation inputs. Only ros2 is wired; grpc is
      reserved. An unknown value falls back to ros2 with a warning.

  camera_frame:
    type: string
    default: ""
    x-group: Sensor overrides
    description: >-
      TF frame the camera publishes in. Empty means the frame_id of the RGB
      images. Scene needs camera to map to place an observation and waits
      rather than guessing a frame. Environment fallback: SCENE_CAMERA_FRAME.

  base_frame:
    type: string
    default: ""
    x-group: Sensor overrides
    description: >-
      The robot's own frame, used to chain the pose and camera extrinsics when
      TF has no camera-to-map transform. Soma's footprint frame takes
      precedence; when the two differ, the mismatch is logged and neither is
      used. Environment fallback: SCENE_BASE_FRAME.

  camera_provider_id:
    type: string
    x-provider: robonix/primitive/camera/rgb
    default: ""
    x-group: Sensor overrides
    description: >-
      Atlas provider that the rgb, depth, intrinsics and camera_extrinsics
      inputs must all come from, when several cameras are registered.

  intrinsics_fallback:
    type: [object, "null"]
    default: null
    x-group: Sensor overrides
    description: >-
      Pinhole intrinsics for a camera that publishes no usable CameraInfo,
      used until one arrives. A deployment needing this is usually missing a
      driver; it exists so a bring-up is not blocked by one. An incomplete
      mapping, or one with a value that is not positive, is ignored with a
      warning.
    required: [width, height, fx, fy, cx, cy]
    properties:
      width:
        type: integer
        description: Must be greater than 0.
      height:
        type: integer
        description: Must be greater than 0.
      fx:
        type: number
        description: Must be greater than 0.
      fy:
        type: number
        description: Must be greater than 0.
      cx:
        type: number
        description: Must be greater than 0.
      cy:
        type: number
        description: Must be greater than 0.
      source:
        type: string
        description: >-
          Label shown in the log; `name` is read when it is absent. none,
          disabled or false turns the fallback off.
    # The strings none, disabled and false, and the boolean false, also turn it
    # off.

  pose_max_age_s:
    type: number
    default: 2.0
    x-group: Sensor overrides
    description: >-
      Seconds after which a pose is no longer used to place an observation. A
      stale pose puts objects where the robot used to be. Environment
      fallback: SCENE_POSE_MAX_AGE_S. 0 means the default; a negative value
      fails activation. Must be greater than 0.

  map_id:
    type: [string, "null"]
    default: null
    x-group: Sensor overrides
    description: >-
      Map partition Scene binds to when Mapping's lifecycle broadcast names
      none. Precedence: Mapping's broadcast, this key, SCENE_MAP_ID, then
      `default`. Leave it unset: a boot then starts a fresh live session that
      the operator names when saving it, and a static value makes a first run
      look like a loaded map. Unless SCENE_RESTORE_ON_START is true, objects
      are kept in a live partition and nothing is restored at boot.
