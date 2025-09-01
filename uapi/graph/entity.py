"""
Entity Graph Module
===================

This module provides the entity graph system for DeepEmbody OS.
It defines the Entity class which represents nodes in the hierarchical
entity graph, along with skill management, primitive operations,
and graph traversal functionality.
"""

# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import uuid
from ..specs.skill_specs import EOS_SKILL_SPECS
from ..log import logger
import dataclasses


def format_primitive_error(primitive_name: str, error_type: str, details: str) -> str:
    """Format primitive errors in a user-friendly way."""
    return f"{error_type}: {details} (primitive: {primitive_name})"


class EntityType(Enum):
    GENERIC = "generic"
    CONTROLLABLE = "controllable"
    COMPUTING = "computing"
    SYSTEM = "system"
    HUMAN = "human"
    ROOM = "room"


class RelationType(Enum):
    # ROOM RELATIONS
    # these are used to construct the main structure of the tree
    # while other relations are used to provide auxiliary information (graph)
    PARENT_OF = "parent_of"
    CHILD_OF = "child_of"


@dataclass
class EntityMetadata:
    name: str = ""
    description: str = ""
    tags: List[str] = field(default_factory=list)


class Entity:

    def __init__(
        self,
        entity_id: str,
        entity_type: EntityType,
        entity_name: str,
        metadata: Optional[EntityMetadata] = None,
    ):

        self.entity_id = entity_id
        self.entity_type = entity_type
        self.entity_name = entity_name
        self.metadata = metadata or EntityMetadata()

        self.relations: Dict[str, List["Entity"]] = {
            rel_type.value: [] for rel_type in RelationType
        }

        self.primitives: List[str] = []
        self.primitive_bindings: Dict[str, callable] = {}

        self.is_active = True
        self.created_at = None
        self.updated_at = None

    def add_primitive(self, primitive: str) -> None:
        if primitive not in self.primitives:
            self.primitives.append(primitive)

    def remove_primitive(self, primitive: str) -> None:
        if primitive in self.primitives:
            self.primitives.remove(primitive)

    def has_primitive(self, primitive: str) -> bool:
        return primitive in self.primitives

    def add_relation(
        self, relation_type: RelationType, target_entity: "Entity"
    ) -> None:
        if target_entity not in self.relations[relation_type.value]:
            self.relations[relation_type.value].append(target_entity)

    def remove_relation(
        self, relation_type: RelationType, target_entity: "Entity"
    ) -> None:
        if target_entity in self.relations[relation_type.value]:
            self.relations[relation_type.value].remove(target_entity)

    def get_relations(self, relation_type: RelationType) -> List["Entity"]:
        return self.relations[relation_type.value].copy()

    def add_parent(self, parent_entity: "Entity") -> None:
        if parent_entity == self:
            raise ValueError("Entity cannot be its own parent")
        self.add_relation(RelationType.PARENT_OF, parent_entity)
        parent_entity.add_relation(RelationType.CHILD_OF, self)

    def add_child(self, child_entity: "Entity") -> None:
        if child_entity == self:
            raise ValueError("Entity cannot be its own child")
        self.add_relation(RelationType.CHILD_OF, child_entity)
        child_entity.add_relation(RelationType.PARENT_OF, self)

    def remove_parent(self, parent_entity: "Entity") -> None:
        self.remove_relation(RelationType.PARENT_OF, parent_entity)
        parent_entity.remove_relation(RelationType.CHILD_OF, self)

    def remove_child(self, child_entity: "Entity") -> None:
        self.remove_relation(RelationType.CHILD_OF, child_entity)
        child_entity.remove_relation(RelationType.PARENT_OF, self)

    def get_parent(self) -> List["Entity"]:
        parent = self.get_relations(RelationType.PARENT_OF)
        if len(parent) > 1:
            raise ValueError("Entity has multiple parents")
        return parent[0] if parent else None

    def get_children(self) -> List["Entity"]:
        return self.get_relations(RelationType.CHILD_OF)

    def is_root(self) -> bool:
        return self.get_parent() is None

    def get_absolute_path(self) -> str:
        path = self.entity_name
        parent = self.get_parent()
        if parent:
            if parent.is_root():
                path = f"/{path}"
            else:
                path = f"{parent.get_absolute_path()}/{path}"
        return path

    def get_entity_by_path(self, path: str) -> Optional["Entity"]:
        """
        Find and return the entity at the given path relative to this entity.
        Path is a string of entity names separated by '/', e.g. 'room1/book1'.
        Returns the target Entity if found, otherwise None.
        """
        # Remove leading/trailing slashes and split
        path_split = [p for p in path.strip("/").split("/") if p]
        if not path_split:
            return self  # path is empty or just '/', return self

        current = self
        for name in path_split:
            children = current.get_children()
            found = False
            for child in children:
                if child.entity_name == name:
                    current = child
                    found = True
                    break
            if not found:
                return None  # Path does not exist
        return current

    def bind_skill(self, primitive_name: str, func: callable) -> None:
        """
        Bind a function to a primitive name for this entity.
        """
        if primitive_name not in EOS_SKILL_SPECS:
            raise ValueError(
                f"Primitive '{primitive_name}' is not a standard primitive."
            )
        self.primitive_bindings[primitive_name] = func
        self.add_primitive(primitive_name)

    def _is_type_match(self, value, expected_type):
        """
        Helper to check if value matches expected_type, where expected_type can be:
        - a type (e.g., str, int)
        - a dict (for dict structure)
        - a list of types/specs (for alternatives)
        - a dataclass
        - an Enum
        """
        import dataclasses
        from enum import Enum
        from typing import List

        # If expected_type is a list, treat as alternatives
        if isinstance(expected_type, list):
            for alt in expected_type:
                if self._is_type_match(value, alt):
                    return True
            return False
        # If expected_type is a dict, value must be dict and match keys/types
        if isinstance(expected_type, dict):
            if not isinstance(value, dict):
                return False
            if set(value.keys()) != set(expected_type.keys()):
                return False
            for k in expected_type:
                if not self._is_type_match(value[k], expected_type[k]):
                    return False
            return True
        # If expected_type is a dataclass
        if dataclasses.is_dataclass(expected_type):
            return isinstance(value, expected_type)
        # If expected_type is an Enum
        if isinstance(expected_type, type) and issubclass(expected_type, Enum):
            return isinstance(value, expected_type)
        # Fallback: direct isinstance
        return isinstance(value, expected_type)

    def _try_cast(self, value, expected_type):
        """
        Try to cast value to expected_type if possible, else return value as is.
        """
        from enum import Enum
        import dataclasses

        # If expected_type is a list, try each alternative
        if isinstance(expected_type, list):
            for alt in expected_type:
                try:
                    return self._try_cast(value, alt)
                except Exception:
                    continue
            raise TypeError(
                f"Value {value!r} does not match any alternative type {expected_type}"
            )
        # If expected_type is a dict, try to cast each field
        if isinstance(expected_type, dict):
            if not isinstance(value, dict):
                raise TypeError(
                    f"Expected dict for {expected_type}, got {type(value).__name__}"
                )
            return {
                k: self._try_cast(value[k], expected_type[k]) for k in expected_type
            }
        # If expected_type is a dataclass, skip casting (user must provide correct type)
        if dataclasses.is_dataclass(expected_type):
            if not isinstance(value, expected_type):
                raise TypeError(
                    f"Expected dataclass {expected_type}, got {type(value).__name__}"
                )
            return value
        # If expected_type is an Enum, try to cast
        if isinstance(expected_type, type) and issubclass(expected_type, Enum):
            if isinstance(value, expected_type):
                return value
            return expected_type(value)
        # Fallback: try direct cast
        return expected_type(value)

    def _check_primitive_args(self, primitive_name: str, kwargs: dict):
        """
        Check if the arguments match the primitive spec.
        Attempts to cast arguments to expected types with warnings before raising errors.
        Now supports multiple alternative types for arguments (e.g., list of types/specs).
        """
        # Log arguments with their types
        arg_info = {k: f"{v} ({type(v).__name__})" for k, v in kwargs.items()}
        logger.debug(
            f"[{self.get_absolute_path()}] checking arguments for primitive '{primitive_name}': {arg_info}"
        )

        spec = EOS_SKILL_SPECS[primitive_name]
        expected_input = spec["input"]

        # Handle case where expected_args is None (no arguments required)
        if expected_input is None:
            if kwargs:
                error_msg = f"Primitive '{primitive_name}' expects no arguments, got {list(kwargs.keys())}"
                logger.error(
                    f"[{self.get_absolute_path()}] argument validation failed: {error_msg}"
                )
                raise ValueError(error_msg) from None
            logger.debug(
                f"[{self.get_absolute_path()}] argument validation passed for primitive '{primitive_name}' (no arguments required)"
            )
            return

        # Handle case where expected_input is a list (multiple alternative argument specs)
        if isinstance(expected_input, list):
            # Try each alternative spec until one matches
            for i, alt_spec in enumerate(expected_input):
                try:
                    if alt_spec is None:
                        # No arguments required
                        if not kwargs:
                            logger.debug(
                                f"[{self.get_absolute_path()}] argument validation passed for primitive '{primitive_name}' (alternative {i+1}: no arguments required)"
                            )
                            return
                        else:
                            continue  # Try next alternative
                    elif isinstance(alt_spec, dict):
                        # Check if this alternative spec matches the provided arguments
                        if set(kwargs.keys()) == set(alt_spec.keys()):
                            # Check argument types and attempt casting if needed
                            for arg_name, expected_type in alt_spec.items():
                                if not self._is_type_match(kwargs[arg_name], expected_type):
                                    # Try to cast the argument to one of the expected types
                                    try:
                                        original_value = kwargs[arg_name]
                                        original_type = type(original_value).__name__
                                        kwargs[arg_name] = self._try_cast(original_value, expected_type)
                                        logger.warning(
                                            f"Type cast for primitive '{primitive_name}' argument '{arg_name}' (alternative {i+1}): "
                                            f"{original_type} -> {type(kwargs[arg_name]).__name__} ({original_value} -> {kwargs[arg_name]})"
                                        )
                                    except (ValueError, TypeError) as cast_error:
                                        continue  # Try next alternative
                            # If we get here, this alternative matched
                            logger.debug(
                                f"[{self.get_absolute_path()}] argument validation passed for primitive '{primitive_name}' (alternative {i+1})"
                            )
                            return
                        else:
                            continue  # Try next alternative
                except Exception:
                    continue  # Try next alternative
            
            # If we get here, no alternative matched
            error_msg = f"Arguments for '{primitive_name}' must match one of the alternatives: {expected_input}, got {list(kwargs.keys())}"
            logger.error(
                f"[{self.get_absolute_path()}] argument validation failed: {error_msg}"
            )
            raise ValueError(error_msg) from None
        
        # Handle case where expected_input is a dict (arguments with types required)
        elif isinstance(expected_input, dict):
            if set(kwargs.keys()) != set(expected_input.keys()):
                error_msg = f"Arguments for '{primitive_name}' must be {list(expected_input.keys())}, got {list(kwargs.keys())}"
                logger.error(
                    f"[{self.get_absolute_path()}] argument validation failed: {error_msg}"
                )
                raise ValueError(error_msg) from None

            # Check argument types and attempt casting if needed
            for arg_name, expected_type in expected_input.items():
                if not self._is_type_match(kwargs[arg_name], expected_type):
                    # Try to cast the argument to one of the expected types
                    try:
                        original_value = kwargs[arg_name]
                        original_type = type(original_value).__name__
                        kwargs[arg_name] = self._try_cast(original_value, expected_type)
                        logger.warning(
                            f"Type cast for primitive '{primitive_name}' argument '{arg_name}': "
                            f"{original_type} -> {type(kwargs[arg_name]).__name__} ({original_value} -> {kwargs[arg_name]})"
                        )
                    except (ValueError, TypeError) as cast_error:
                        error_msg = f"Argument '{arg_name}' for '{primitive_name}' must be {expected_type}, got {type(kwargs[arg_name]).__name__}"
                        logger.error(
                            f"[{self.get_absolute_path()}] type validation failed: {error_msg}"
                        )
                        raise TypeError(error_msg) from None
        else:
            error_msg = f"Invalid input spec type for '{primitive_name}': {type(expected_input)}"
            logger.error(
                f"[{self.get_absolute_path()}] argument validation failed: {error_msg}"
            )
            raise ValueError(error_msg) from None

        logger.debug(
            f"[{self.get_absolute_path()}] argument validation passed for primitive '{primitive_name}'"
        )

    def _check_primitive_returns(self, primitive_name: str, result):
        """
        Check if the return value matches the primitive spec, supporting dataclass, dict, list, and enum recursively.
        """
        logger.debug(
            f"[{self.get_absolute_path()}] checking return value for primitive '{primitive_name}': {result}"
        )

        spec = EOS_SKILL_SPECS[primitive_name]
        expected_output = spec["output"]

        def recursive_type_check(value, expected_type):
            # Handle Any type - accept anything
            if expected_type is Any:
                return True
            # Handle dataclass
            if dataclasses.is_dataclass(expected_type):
                if not isinstance(value, expected_type):
                    return False
                for field in dataclasses.fields(expected_type):
                    v = getattr(value, field.name)
                    t = field.type
                    if not recursive_type_check(v, t):
                        return False
                return True
            # Handle dict
            if isinstance(expected_type, dict):
                if not isinstance(value, dict):
                    return False
                if set(value.keys()) != set(expected_type.keys()):
                    return False
                for k in expected_type:
                    if not recursive_type_check(value[k], expected_type[k]):
                        return False
                return True
            # Handle list/tuple (only check type, not length)
            if getattr(expected_type, "__origin__", None) in (list, List):
                if not isinstance(value, list):
                    return False
                elem_type = expected_type.__args__[0]
                return all(recursive_type_check(v, elem_type) for v in value)
            # Handle dict with subscripted generics (e.g., Dict[str, Any])
            if getattr(expected_type, "__origin__", None) is dict:
                if not isinstance(value, dict):
                    return False
                # For Dict[str, Any], we can't check the actual types at runtime
                # So we just check that it's a dict
                return True
            # Handle tuple with subscripted generics (e.g., Tuple[float, float, float], Tuple[Any, Any])
            if getattr(expected_type, "__origin__", None) is tuple:
                if not isinstance(value, tuple):
                    return False
                # Check tuple length matches expected length
                if len(value) != len(expected_type.__args__):
                    return False
                # Check each element of the tuple
                for i, elem_type in enumerate(expected_type.__args__):
                    if not recursive_type_check(value[i], elem_type):
                        return False
                return True
            # Handle enum
            if isinstance(expected_type, type) and issubclass(expected_type, Enum):
                return isinstance(value, expected_type)
            # Fallback: direct isinstance
            return isinstance(value, expected_type)

        if not recursive_type_check(result, expected_output):
            error_msg = f"Return value for '{primitive_name}' does not match expected type {expected_output}"
            logger.error(
                f"[{self.get_absolute_path()}] return type validation failed: {error_msg}"
            )
            raise TypeError(error_msg) from None

        logger.debug(
            f"[{self.get_absolute_path()}] return value validation passed for primitive '{primitive_name}'"
        )

    def __getattr__(self, name):
        # https://www.sefidian.com/2021/06/06/python-__getattr__-and-__getattribute__-magic-methods/
        # getattr is called when an attribute is not found in the object, while __getattribute__ is called no matter found or not
        # we use getattr to "bind" primitives to the entity - wheatfox
        if name in self.primitive_bindings:

            def wrapper(**kwargs):
                logger.debug(
                    f"[{self.get_absolute_path()}] calling primitive {name} with kwargs {kwargs}"
                )
                try:
                    # Get the actual function from primitive_bindings
                    func = self.primitive_bindings[name]
                    
                    # Check if the function expects self_entity as first positional argument
                    import inspect
                    sig = inspect.signature(func)
                    params = list(sig.parameters.keys())
                    
                    if len(params) > 0 and params[0] == 'self_entity':
                        # Function expects self_entity as first positional argument
                        # Convert kwargs to positional args for the first parameter
                        if 'self_entity' in kwargs:
                            self_entity = kwargs.pop('self_entity')
                        else:
                            self_entity = self
                        
                        # Call the function with self_entity as first positional argument
                        result = func(self_entity, **kwargs)
                    else:
                        # Function doesn't expect self_entity, use normal validation
                        # Note: We don't inject self_entity here to avoid breaking existing capability signatures
                        # self_entity injection should be handled at the decorator level for skills that need it
                        self._check_primitive_args(name, kwargs)
                        result = func(**kwargs)
                        self._check_primitive_returns(name, result)
                    
                    return result
                except (ValueError, TypeError) as e:
                    logger.error(
                        f"[{self.get_absolute_path()}] primitive '{name}' execution failed: {str(e)}"
                    )
                    # Create a custom exception with better formatting
                    error_msg = format_primitive_error(name, type(e).__name__, str(e))
                    custom_exc = type(e)(error_msg)
                    raise custom_exc from None

            return wrapper
        
        # Support for skill and capability calling with self_entity.skl_xxx and self_entity.cap_xxx
        # Note: The actual function names in skill/__init__.py may not start with cap_ or skl_
        # They are bound to standard names through entity.bind_skill() method
        if name.startswith('skl_') or name.startswith('cap_'):
            def wrapper(**kwargs):
                # Inject self_entity into the kwargs for skills and capabilities that support it
                kwargs['self_entity'] = self
                
                # First, try to find the function in primitive bindings (standard names)
                if name in self.primitive_bindings:
                    logger.debug(
                        f"[{self.get_absolute_path()}] calling primitive {name} with self_entity injection"
                    )
                    try:
                        # For skills and capabilities that support self_entity, we need to handle it specially
                        # Check if the function accepts self_entity parameter
                        func = self.primitive_bindings[name]
                        import inspect
                        sig = inspect.signature(func)
                        params = list(sig.parameters.keys())
                        
                        if len(params) > 0 and params[0] == 'self_entity':
                            # Function expects self_entity as first positional argument
                            self_entity = kwargs.pop('self_entity')
                            result = func(self_entity, **kwargs)
                        elif 'self_entity' in sig.parameters:
                            # Function supports self_entity as keyword argument
                            result = func(**kwargs)
                        else:
                            # Function doesn't support self_entity, remove it from kwargs
                            kwargs.pop('self_entity', None)
                            self._check_primitive_args(name, kwargs)
                            result = func(**kwargs)
                            self._check_primitive_returns(name, result)
                        return result
                    except (ValueError, TypeError) as e:
                        logger.error(
                            f"[{self.get_absolute_path()}] primitive '{name}' execution failed: {str(e)}"
                        )
                        error_msg = format_primitive_error(name, type(e).__name__, str(e))
                        custom_exc = type(e)(error_msg)
                        raise custom_exc from None
                
                # If not found in primitive bindings, try to find in skill module
                # The actual function names in skill/__init__.py may not match the standard names
                try:
                    from skill import __all__
                    import skill
                    
                    # Try to find the function by standard name first
                    if name in __all__:
                        func = getattr(skill, name)
                        logger.debug(
                            f"[{self.get_absolute_path()}] calling {name} from skill module with self_entity injection"
                        )
                        return func(**kwargs)
                    
                    # If not found by standard name, try to find by actual function name
                    # This handles cases where the actual function doesn't start with cap_ or skl_
                    for actual_func_name in __all__:
                        # Check if this function is bound to the standard name
                        # We can't easily determine this mapping, so we'll try calling
                        # the function and see if it works
                        try:
                            func = getattr(skill, actual_func_name)
                            # Try calling the function to see if it's the right one
                            # This is a bit hacky but works for the current use case
                            if hasattr(func, '_is_skill') or hasattr(func, '_is_capability'):
                                logger.debug(
                                    f"[{self.get_absolute_path()}] trying {actual_func_name} for {name}"
                                )
                                # We found a skill/capability function, but we can't easily
                                # determine if it's the right one for this standard name
                                # For now, we'll skip this approach and rely on proper binding
                                pass
                        except Exception:
                            continue
                            
                except (ImportError, AttributeError) as e:
                    logger.debug(f"[{self.get_absolute_path()}] {name} not found in skill module: {e}")
                
                # If not found, raise AttributeError
                raise AttributeError(
                    f"'{type(self).__name__}' object has no attribute '{name}', "
                    f"and '{name}' is not available as a skill or capability. "
                    f"Available primitives for {self.get_absolute_path()}: {self.primitives}"
                )
            
            return wrapper
        
        raise AttributeError(
            f"'{type(self).__name__}' object has no attribute '{name}', or this primitive is not bound, available primitives for {self.get_absolute_path()}: {self.primitives}"
        )


class Room(Entity):

    def __init__(
        self,
        room_id: str,
        room_name: str,
        metadata: Optional[EntityMetadata] = None,
    ):
        super().__init__(
            entity_id=room_id,
            entity_type=EntityType.ROOM,
            entity_name=room_name,
            metadata=metadata,
        )

        self.capacity: Optional[int] = None
        self.room_type: str = "generic"  # special attributes for Room
        self.is_accessible: bool = True


def create_generic_entity(entity_name: str, **kwargs) -> Entity:
    return Entity(uuid.uuid4(), EntityType.GENERIC, entity_name, **kwargs)


def create_controllable_entity(entity_name: str, **kwargs) -> Entity:
    return Entity(uuid.uuid4(), EntityType.CONTROLLABLE, entity_name, **kwargs)


def create_computing_entity(entity_name: str, **kwargs) -> Entity:
    return Entity(uuid.uuid4(), EntityType.COMPUTING, entity_name, **kwargs)


def create_human_entity(entity_name: str, **kwargs) -> Entity:
    return Entity(uuid.uuid4(), EntityType.HUMAN, entity_name, **kwargs)


def create_room_entity(room_name: str, room_type: str = "generic", **kwargs) -> Room:
    room = Room(uuid.uuid4(), room_name, **kwargs)
    room.room_type = room_type
    return room


def create_root_room() -> Room:
    return create_room_entity("/")
