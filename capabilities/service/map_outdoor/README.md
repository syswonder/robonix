# Outdoor route mapping

These six RPC capabilities share `outdoor_map/srv/*.srv`. gRPC and MCP use the generated request/response types. `request_json` must encode an object. An unsuccessful response has `ok=false` and a readable `detail`; a successful response has JSON in `result_json`.

| RPC | Request object | Result |
|---|---|---|
| `get_map` | `{}` for the active map; `{"draft":true}` for the draft; `{"map_id":"id"}` for a saved artifact | Snapshot below |
| `capture` | `{"action":"start","new_map":true}`; omit new_map to supplement. Other actions: pause, resume, stop, cancel, discard. Recover: `{"action":"recover","session_id":"id"}` | Recording status |
| `status` | `{}` | Input freshness, recording state, active map identity, live trajectory and valid GNSS position |
| `preview_merge` | `{}` after recording finishes | `preview_id`, `base_version`, `draft_version`, combined network, candidates and overlaps |
| `apply_merge` | `{"preview_id":"...","decisions":{"join-0":"connect"},"choices":{"overlap-0":"old"}}` | Reviewed draft snapshot |
| `plan_route` | `{"map_id":"id","version":"hash","start":{"x":0,"y":0},"goal":{"x":2,"y":0},"blocked_road_ids":[]}` | Reference path and ordered gait/quality intervals |

All connection candidates require a `connect` or `separate` decision. All overlaps require an `old` or `new` gait-record choice, including equal-gait overlaps for provenance review. Invalid or stale preview IDs are refused. Separating roads preserves their topology; overlap geometry follows the old measurement. Cancel closes an open recording, marks it interrupted and retains its raw events without building a draft. Recording, pause/resume, cancellation and raw recovery never issue robot commands.

A snapshot contains `schema_version`, `map_type="outdoor"`, immutable `map_id`, `version` (content hash), `parent_map_id`, `frame_id="map"`, `units="meters"`, fixed `origin_wgs84={latitude,longitude}`, `network`, `recordings`, filtered tracks and processing report. Synthetic snapshots set `example_data=true`.

`network.nodes` contain `id,x,y`. `network.roads` contain `id,from,to,points,length_m,bidirectional,gait,gait_code,quality,grade,low_accuracy,sources`. Roads connect only through shared node IDs. Geometry uses east X/north Y in metres relative to the fixed WGS84 origin; the position reference is the GNSS antenna. Gait values are `flat`, `stairs`, or `unknown`, with the actual feedback code retained. Receiver qualities 4/5/2 map to `fixed`/`float`/`dgps`; DGPS roads participate in planning and carry `low_accuracy=true`.

Status separates `ready` (all three input streams are fresh) from `position_valid` and `position_grade`. `inputs` includes freshness, reception age and source for fix, RTK status and actual gait. `session.state` is recording, paused, finished or interrupted. A missing input is absent from `inputs`; clients must show it as unavailable. A stale or unusable position yields `pose=null`. `live_track` carries segment numbers; clients must not join different segments. `heading=null` means no orientation arrow may be drawn.

Planning requires the exact map ID/version and both endpoints within two metres of measured roads. Omit `start` to use fresh GNSS. Results contain `reference_path`, `segment_roads`, `road_ids`, projected `start` and `goal`, `length_m`, `blocked_road_ids`, map ID/version/origin/frame and `intervals`. Each interval has `road_id,start_m,end_m,gait,gait_code,grade,low_accuracy,sources`; distances and interval order follow the requested travel direction, including reverse travel. `low_accuracy` summarizes the route and `execution_enabled=false` identifies a preview, without requesting motion.

Standard `robonix/service/map` save/list/load/delete/get_mode/lifecycle capabilities manage these artifacts. Save publishes a new immutable ID and leaves the active map unchanged; load explicitly activates a saved route network. Initial-pose seeding is unsupported. `MapLifecycle.generation` tracks frame lifecycle separately from the content hash. An outdoor load does not imply SLAM relocalization, and there is no occupancy-grid capability.
