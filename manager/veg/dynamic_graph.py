from entity import *
import threading
import time
from typing import Set, Dict, List
from dataclasses import dataclass
from enum import Enum


class UpdateEvent(Enum):
    ENTITY_ADDED = "entity_added"
    ENTITY_REMOVED = "entity_removed"
    RELATION_CHANGED = "relation_changed"
    POSITION_CHANGED = "position_changed"
    STATE_CHANGED = "state_changed"


@dataclass
class GraphUpdate:
    event_type: UpdateEvent
    entity_id: str
    entity_name: str
    details: Dict = None
    timestamp: float = None


class DynamicGraph:
    def __init__(self, root_room: Room, update_interval: float = 1.0):
        self.root_room = root_room
        self.update_interval = update_interval
        self.update_daemon = None
        self.is_running = False
        self.update_count = 0

        self.entity_states: Dict[str, Dict] = {}
        self.update_history: List[GraphUpdate] = []
        self.max_history_size = 100

        self._lock = threading.Lock()

        # self._initialize_entity_state(root_room)

    def _initialize_entity_state(self, entity: Entity):
        with self._lock:
            self.entity_states[entity.entity_id] = {
                "position": (entity.position.x, entity.position.y, entity.position.z),
                "is_active": entity.is_active,
                "children_count": len(entity.get_children()),
                "primitives": (
                    entity.primitives.copy() if hasattr(entity, "primitives") else []
                ),
            }

            for child in entity.get_children():
                self._initialize_entity_state(child)

    def _traverse_entities(
        self, entity: Entity, visited: Set[str] = None
    ) -> List[Entity]:
        if visited is None:
            visited = set()

        if entity.entity_id in visited:
            return []

        visited.add(entity.entity_id)
        entities = [entity]

        for child in entity.get_children():
            entities.extend(self._traverse_entities(child, visited))

        return entities

    def _check_entity_changes(self, entity: Entity) -> List[GraphUpdate]:
        updates = []
        entity_id = entity.entity_id

        if entity_id not in self.entity_states:
            self._initialize_entity_state(entity)
            updates.append(
                GraphUpdate(
                    event_type=UpdateEvent.ENTITY_ADDED,
                    entity_id=entity_id,
                    entity_name=entity.entity_name,
                    details={"entity_type": entity.entity_type.value},
                    timestamp=time.time(),
                )
            )
            return updates

        current_state = self.entity_states[entity_id]

        current_pos = (entity.position.x, entity.position.y, entity.position.z)
        if current_pos != current_state["position"]:
            updates.append(
                GraphUpdate(
                    event_type=UpdateEvent.POSITION_CHANGED,
                    entity_id=entity_id,
                    entity_name=entity.entity_name,
                    details={
                        "old_position": current_state["position"],
                        "new_position": current_pos,
                    },
                    timestamp=time.time(),
                )
            )
            current_state["position"] = current_pos

        if entity.is_active != current_state["is_active"]:
            updates.append(
                GraphUpdate(
                    event_type=UpdateEvent.STATE_CHANGED,
                    entity_id=entity_id,
                    entity_name=entity.entity_name,
                    details={
                        "old_state": current_state["is_active"],
                        "new_state": entity.is_active,
                    },
                    timestamp=time.time(),
                )
            )
            current_state["is_active"] = entity.is_active

        current_children_count = len(entity.get_children())
        if current_children_count != current_state["children_count"]:
            updates.append(
                GraphUpdate(
                    event_type=UpdateEvent.RELATION_CHANGED,
                    entity_id=entity_id,
                    entity_name=entity.entity_name,
                    details={
                        "old_children_count": current_state["children_count"],
                        "new_children_count": current_children_count,
                    },
                    timestamp=time.time(),
                )
            )
            current_state["children_count"] = current_children_count

        current_primitives = (
            entity.primitives.copy() if hasattr(entity, "primitives") else []
        )
        if current_primitives != current_state["primitives"]:
            updates.append(
                GraphUpdate(
                    event_type=UpdateEvent.STATE_CHANGED,
                    entity_id=entity_id,
                    entity_name=entity.entity_name,
                    details={
                        "old_primitives": current_state["primitives"],
                        "new_primitives": current_primitives,
                    },
                    timestamp=time.time(),
                )
            )
            current_state["primitives"] = current_primitives

        return updates

    def _update_entity_states(self):
        with self._lock:
            all_entities = self._traverse_entities(self.root_room)

            for entity in all_entities:
                updates = self._check_entity_changes(entity)
                self.update_history.extend(updates)

            if len(self.update_history) > self.max_history_size:
                self.update_history = self.update_history[-self.max_history_size :]

            current_entity_ids = {entity.entity_id for entity in all_entities}
            deleted_entity_ids = set(self.entity_states.keys()) - current_entity_ids

            for entity_id in deleted_entity_ids:
                entity_name = self.entity_states[entity_id].get("name", "Unknown")
                self.update_history.append(
                    GraphUpdate(
                        event_type=UpdateEvent.ENTITY_REMOVED,
                        entity_id=entity_id,
                        entity_name=entity_name,
                        timestamp=time.time(),
                    )
                )
                del self.entity_states[entity_id]

    def update(self):
        while self.is_running:
            try:
                self._update_entity_states()
                self.update_count += 1

                if self.update_history:
                    latest_update = self.update_history[-1]
                    print(
                        f"Update #{self.update_count}: {latest_update.event_type.value} - {latest_update.entity_name}"
                    )

                time.sleep(self.update_interval)

            except Exception as e:
                print(f"Error in dynamic graph update: {e}")
                time.sleep(self.update_interval)

    def start(self):
        if self.is_running:
            print("Dynamic graph is already running")
            return

        self.is_running = True
        self.update_daemon = threading.Thread(target=self.update, daemon=True)
        self.update_daemon.start()
        print(f"Dynamic graph started with {self.update_interval}s update interval")

    def stop(self):
        if not self.is_running:
            print("Dynamic graph is not running")
            return

        self.is_running = False
        if self.update_daemon and self.update_daemon.is_alive():
            self.update_daemon.join(timeout=2.0)
        print("Dynamic graph stopped")

    def get_update_history(self) -> List[GraphUpdate]:
        with self._lock:
            return self.update_history.copy()

    def get_entity_count(self) -> int:
        with self._lock:
            return len(self.entity_states)

    def get_active_entities(self) -> List[Entity]:
        with self._lock:
            return [
                entity
                for entity_id, entity in self.entity_states.items()
                if entity["is_active"]
            ]

    def get_entity_info(self, entity_id: str) -> Dict:
        with self._lock:
            return self.entity_states.get(entity_id, {}).copy()

    def set_update_interval(self, interval: float):
        if interval <= 0:
            raise ValueError("Update interval must be positive")
        self.update_interval = interval
        print(f"Update interval changed to {interval}s")
