# Runtime configuration accepted by the Scene service.
#
# This documents the mapping passed as the service instance's `config:` value
# in a deployment manifest, delivered through Driver(CMD_INIT) by `rbnx boot`.
# It is not loaded as a schema. Environment variables provide fallbacks for
# standalone use; instance config takes priority over them.
#
# Unknown keys under `perception.dualmap` fail Driver(CMD_INIT), naming the key
# and listing what is accepted. Unknown keys elsewhere are logged as ignored and
# the service starts.
#
#
# ─── Read this first: almost nothing here has to be set ──────────────────────
#
# Scene finds its inputs through Atlas. When a camera capability is registered,
# the auto-discovery loop resolves the RGB, depth, intrinsics and transform
# topics itself and logs what it bound to:
#
#     [scene] rgb ← atlas: topic=/head_front_camera/rgb/image_raw contract=...
#
# So the whole sensor half of this file is for the case where that does not
# work. A deployment that spells out topics Atlas already knows about has two
# sources of truth and will eventually disagree with itself.
#
# What actually has to be set is four keys, all under `perception.dualmap`, and
# only when that backend is selected. Their defaults come from DualMap upstream
# and are sized for a dataset replay that maps every frame; a robot mapping
# keyframes at walking pace sees each object a handful of times, and at the
# upstream defaults one 180-second office run went from 65 tracks to 1 --- the
# map emptied out behind the robot.
#
#     perception.dualmap.classes                 the deployment's vocabulary
#     perception.dualmap.stable_num              observations to become stable
#     perception.dualmap.keyframe_translation_m  when a frame is mapped
#     perception.dualmap.keyframe_rotation_deg   likewise
#
# Everything else in this file has a working default. The three sections are
# ordered accordingly: what to set, what to set occasionally, and what to leave
# alone unless something is wrong.

config:

  # ═══ 1. Set this ══════════════════════════════════════════════════════════

  perception:

    # string, default: concept_graphs (environment fallback
    # SCENE_PERCEPTION_BACKEND). Which open-vocabulary mapper runs on the RGB-D
    # stream: concept_graphs or dualmap. Both take the same inputs and feed the
    # same ObjectRegistry, so nothing above the detector changes.
    #
    # The backend also has to be known when the image is built --- DualMap
    # brings its own checkout and weights --- so build and boot with the same
    # value. Starting a mismatched image fails activation with
    # "DualMap root /opt/dualmap has no config/ directory", which reads like a
    # configuration problem and is not.
    backend: dualmap

    # mapping, default: {}. Passed to the DualMap backend; ignored by the other.
    dualmap:

      # REQUIRED. list of strings, default: DualMap's 101-name domestic list.
      # The YOLO-World vocabulary. A detector can only answer with a name it
      # was given: on an office world the domestic list cost 0.41 label
      # accuracy against 0.82 with a 40-name office list, calling the cabinet
      # a sink and the chair an ironing board. Wider than the room is fine ---
      # it keeps the score about recognition rather than about a lookup.
      classes: [chair, table, sofa, door, window, lamp, monitor, shelf]

      # REQUIRED. integer, default: 8. Observations before a track counts as
      # stable. Match it to how often the robot actually sees a thing; at the
      # upstream 8 the map empties out behind a walking robot.
      stable_num: 3

      # REQUIRED. finite float metres, default: 0.1. Map a frame only after
      # this much travel. Feeding every frame fragments objects; a frame taken
      # without motion adds nothing.
      keyframe_translation_m: 0.15

      # REQUIRED. finite float degrees, default: 3.0. Or after this much turn.
      keyframe_rotation_deg: 5.0

  # ═══ 2. Occasionally ══════════════════════════════════════════════════════

  # perception, continued.
  #
  #   profile: lite | full | annotate     default lite (fallback SCENE_PROFILE)
  #     lite      the small model set, about 4 GB of video memory
  #     full      the paper-tier set (SAM-L + CLIP ViT-H-14) from SCENE_MODELS_DIR
  #     annotate  recognition off; manual regions and geometric queries only
  #
  #   period_s: finite float seconds, backend default.
  #     How often the perception loop runs. DualMap also gates on the keyframe
  #     rule above, so a shorter period costs it little; on concept_graphs it
  #     costs a full detect/segment/encode pass.
  #
  #   confidence_threshold: finite float in [0, 1], backend default.
  #     Detections below this are dropped before association.
  #
  #   max_detections: integer, backend default.
  #     Upper bound on detections carried from one frame.
  #
  # perception.concept_graphs: mapping, default {}. The default backend's own
  # knobs --- thirty-nine of them, more than DualMap has. An unknown key fails
  # the boot naming it, the same way the DualMap block does. Env fallbacks for
  # the most-tuned ones are SCENE_CG_*.
  #
  #   Merge and identity. Reach for these first: a run of office shelving
  #   coming back as several objects means the association gates are tighter
  #   than the pose error between views, and these are what the detector's own
  #   comment calls the knobs for "the desk chair/table split and object
  #   dedup", tunable on a running robot without a rebuild.
  #
  #     merge_threshold: 0.85           point overlap two objects need to merge
  #     max_merge_dist_m: 1.5           centroid gate on the per-tick merge
  #     merge_overlap_thresh: 0.5       the periodic pass's overlap gate
  #     merge_visual_sim_thresh: 0.65   ...and its visual-similarity gate
  #     merge_text_sim_thresh: 0.0      ...and its text-similarity gate
  #     same_class_merge_dist_m: 0.4    two of one class this close are one
  #     same_class_merge_interval_ticks: 10
  #     cross_class_centroid_max_m: 0.5 two of different classes, likewise
  #     cross_class_iou_thresh: 0.3
  #     cross_class_overlap_thresh: 0.5
  #     cross_class_merge_interval_ticks: 10
  #     denoise_interval_ticks: 10      cadence of the denoise pass
  #     merge_overlap_interval_ticks: 10
  #
  #   Association --- how a detection is matched to an existing object.
  #
  #     association: voxel_vote         or the older sim-sum path
  #     assoc_voxel_size_m: 0.04        must exceed the pose error between views
  #     assoc_geo_weight: 0.8           geometry's share of the score
  #     assoc_feat_weight: 0.2          the feature's share
  #     assoc_threshold: 0.4            score to join rather than start new
  #     spatial_sim_type: iou           iou | giou | overlap and the oriented
  #                                     variants; 'overlap' is concept-graphs's
  #                                     canonical choice
  #     match_method: sim_sum
  #     phys_bias: 0.0
  #
  #   Point clouds --- what geometry an object is allowed to be made of.
  #
  #     downsample_voxel_size: 0.025    Open3D voxel size, metres
  #     min_points_threshold: 50        a detection with fewer is dropped
  #     obj_min_points: 20              an object with fewer is dropped
  #     obj_pcd_max_points: 5000        per-object cap; the cloud is
  #                                     downsampled once it is passed
  #     obj_min_detections: 1           detections before an object is real
  #     floor_z_m: 0.0                  where the floor plane is
  #     dbscan_remove_noise: true       drop sparse outlier points
  #     dbscan_eps: 0.1                 cluster radius, metres
  #     dbscan_min_points: 10
  #     per_detection_dbscan: false     denoise each detection, not each object
  #
  #   Labels and features.
  #
  #     label_vote: true                vote a label across observations rather
  #                                     than taking the latest
  #     feature_bank_size: 32           CLIP features kept per object
  #     feature_area_ratio: 0.5         mask area share a view needs to count
  #     representative_by_text: true    pick the representative view by text
  #                                     similarity rather than by size
  #
  #   Visibility --- whether a miss means gone or means occluded.
  #
  #     visibility_depth_margin_m: 0.1  measured surface must be this far
  #                                     behind the object to count as a miss
  #     visibility_min_clear_samples: 3
  #     visibility_min_clear_fraction: 0.6
  #     visibility_miss_ticks: 3        misses before an object goes missing
  #
  # perception.dualmap, continued. These are DualMap's own defaults and are
  # worth reading before changing:
  #
  #   keyframe_time_s: 5.0        map a frame anyway after this long
  #   sim_threshold: 1.2          cos(CLIP) + point overlap to join a track
  #   downsample_voxel_size: 0.02 the radius "overlap" counts within. Must
  #                               exceed the SLAM pose error between keyframes,
  #                               or two views of one object never overlap and
  #                               every keyframe starts a new track.
  #   merge_every_keyframes: 20   local-map self-merge cadence; 0 disables
  #   merge_sim_threshold: 0.9    overlap two tracks need to merge. Upstream's
  #                               0.9 never fires under a robot's pose error.
  #   active_window_size: 10      how many recent frames count as active
  #   max_pending_count: 5        rounds an unstable track survives outside it.
  #                               Raising this also delays promotion.
  #   min_observations: 1         keyframes before a track enters the registry
  #   stable_only: false          also require DualMap's own stable flag
  #   keep_unknown: false         keep FastSAM segments YOLO-World could not name
  #   use_fastsam: false          follows keep_unknown; FastSAM only adds those
  #   global_map: false           run DualMap's abstract map as well. It merges
  #                               across classes by top-down 2D overlap --- the
  #                               mechanism aimed at one workstation reported as
  #                               tv + speaker + desk --- but keeps only
  #                               low-mobility anchors and drops every other
  #                               stable track once it leaves view. An inventory
  #                               wants this off; a navigation memory wants it on.
  #   floor_gate: false           drop tracks whose points all lie within 5 cm
  #                               of the floor. Off because it also drops rugs.
  #   floor_z_m: 0.0              where that floor is
  #   device: null                torch device; cuda when available, else cpu

  # optional integer port, default 50107 (fallback SCENE_WEB_PORT). 0 disables
  # the web interface.
  web_port: 50107

  # ═══ 3. Leave alone unless something is wrong ═════════════════════════════

  # These describe the sensor path, which Atlas normally resolves on its own.
  # Setting them is how a deployment overrides that discovery; it is not part
  # of a normal manifest.
  #
  #   observations: list of {kind, topic, msg}, no default.
  #     Which ROS 2 topics ingest subscribes to. `kind` is one of rgb, depth,
  #     lidar2d, pose, odom. Without an rgb and a depth entry --- discovered or
  #     declared --- the perception loop has nothing to read and says so rather
  #     than starting empty.
  #
  #   transport: string, default ros2.
  #
  #   camera_frame: TF frame, default "" (fallback SCENE_CAMERA_FRAME).
  #     The frame the camera publishes in. Scene needs camera->map to place an
  #     observation; with neither this nor discovery it waits rather than
  #     guessing a frame.
  #
  #   base_frame: TF frame, default "" (fallback SCENE_BASE_FRAME).
  #     The robot's own frame, for the footprint and for reachability.
  #
  #   camera_provider_id: string, default "".
  #     Atlas provider id to read intrinsics from when several are registered.
  #
  #   intrinsics_fallback: mapping, default none.
  #     Pinhole intrinsics for a camera that publishes no CameraInfo. A
  #     deployment needing this is usually missing a driver; it exists so a
  #     bring-up is not blocked by one.
  #
  #   pose_max_age_s: finite float seconds, default 2.0
  #                   (fallback SCENE_POSE_MAX_AGE_S).
  #     A pose older than this is not used to place an observation. A stale
  #     pose puts objects where the robot used to be.
  #
  #   map_id: string, default none (fallback SCENE_MAP_ID).
  #     The map this instance binds to. Leave it unset: a boot then starts a
  #     fresh live session that the operator names when they save it, and a
  #     static default makes a first run look like a loaded map.
