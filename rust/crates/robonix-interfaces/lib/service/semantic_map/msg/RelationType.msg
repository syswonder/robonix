# RelationType enum — predicates the scene relation engine can publish.
# CHILD_OF / ON_TOP / INSIDE / NEAR existed pre-v1; REACHABLE_BY is added
# for `system/scene`'s reachability predicate (currently a stub gated on
# distance to the robot's gripper, will be replaced by Soma's kinematics
# adapter once that lands).
uint8 CHILD_OF = 0
uint8 ON_TOP = 1
uint8 INSIDE = 2
uint8 NEAR = 3
uint8 CUSTOM = 4
uint8 REACHABLE_BY = 5
uint8 type
string custom_type  # Only used when type == CUSTOM
