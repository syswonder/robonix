# uapi package

# Export runtime functionality
from .runtime.runtime import Runtime, get_runtime
from .runtime.action import action, EOS_TYPE_ActionResult, set_runtime, action_print

__all__ = [
    'Runtime',
    'get_runtime',
    'action',
    'EOS_TYPE_ActionResult',
    'set_runtime',
    'action_print'
] 