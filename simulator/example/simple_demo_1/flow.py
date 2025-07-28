from uapi.specs.skill_specs import EntityPath
from uapi.runtime.flow import flow, EOS_TYPE_FlowResult, get_runtime, flow_print

@flow
def move_a_to_b(a: EntityPath, b: EntityPath) -> EOS_TYPE_FlowResult:
    runtime = get_runtime()

    A = runtime.get_graph().get_entity_by_path(a)
    B = runtime.get_graph().get_entity_by_path(b)

    if A is None:
        flow_print(f"(move_a_to_b) entity A not found at path: {a}")
        return EOS_TYPE_FlowResult.FAILURE

    if B is None:
        flow_print(f"(move_a_to_b) entity B not found at path: {b}")
        return EOS_TYPE_FlowResult.FAILURE

    b_pos = B.c_space_getpos()
    flow_print(f"(move_a_to_b) entity B position: {b_pos}")

    result = A.c_space_move(x=b_pos["x"], y=b_pos["y"], z=b_pos["z"])
    flow_print(f"(move_a_to_b) move result: {result}")

    return EOS_TYPE_FlowResult.SUCCESS


@flow
def simple_test_flow() -> EOS_TYPE_FlowResult:
    runtime = get_runtime()
    graph = runtime.get_graph()

    flow_print(f"(simple_test_flow) root entity: {graph.get_absolute_path()}")

    children = graph.get_children()
    flow_print(f"(simple_test_flow) root children: {[child.get_absolute_path() for child in children]}")

    return EOS_TYPE_FlowResult.SUCCESS
