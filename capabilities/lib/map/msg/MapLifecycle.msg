# Map identity + lifecycle broadcast from the mapping service.
#
# Published latched (transient_local) so late-joining consumers (e.g. the
# scene service) learn the current map identity at startup, and re-published
# on every lifecycle transition (init / load / reset / mode switch; save
# does not change the live identity — its re-key semantics are a later,
# separate design).
#
# The (map_id, generation) pair is the map-frame identity key: consumers
# holding map-frame coordinates must treat a change in EITHER field as
# "previous coordinates are no longer anchored" — reset_map keeps map_id
# but moves the map origin, which only generation makes visible.
string map_id      # named-map id currently bound; empty when ephemeral (no map_id configured)
string mode        # "mapping" or "localization"
uint64 generation  # monotonic; +1 when the map origin may have changed: mapping-mode
                   # session start/load and reset_map. NEVER bumped by a localization
                   # load (stable frame is the point) or by a mode switch.
