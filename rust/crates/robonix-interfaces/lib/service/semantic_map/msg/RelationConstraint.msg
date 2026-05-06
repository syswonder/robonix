# A "this object stands in <predicate> with <target>" filter, used by
# scene/query. Either target_object_id (exact) or target_cls (any
# object of that class) — never both.
#
# Sentinels: empty target_object_id + empty target_cls == match any
# target (the predicate alone is enough).
string predicate          # "on" | "inside" | "near" | "reachable_by"
string target_object_id
string target_cls
