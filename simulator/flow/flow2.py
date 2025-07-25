# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import sys
sys.path.append(".")

from manager.graph.entity import *

root = None


def sim_gen_graph():
    global root
    root = create_root_room()
    print(f"ID = {root.entity_id}, path = {root.get_absolute_path()}")

    room: Room = create_room_entity("room")
    root.add_child(room)
    print(f"ID = {room.entity_id}, path = {room.get_absolute_path()}")

    ranger: Entity = create_controllable_entity("ranger")
    room.add_child(ranger)
    print(f"ID = {ranger.entity_id}, path = {ranger.get_absolute_path()}")

    book1: Entity = create_generic_entity("book1")
    room.add_child(book1)
    print(f"ID = {book1.entity_id}, path = {book1.get_absolute_path()}")

    book1.bind_skill("getpos", lambda: {"x": -2.2, "y": 1.8, "z": 0.1})

    plant_pot: Entity = create_generic_entity("plant_pot")
    room.add_child(plant_pot)
    print(f"ID = {plant_pot.entity_id}, path = {plant_pot.get_absolute_path()}")

    plant_pot.bind_skill("getpos", lambda: {"x": 2.0, "y": -2.0, "z": 0.2})


def sim_run_flow():
    book1 = root.get_entity_by_path("room/book1")
    ranger = root.get_entity_by_path("room/ranger")
    plant_pot = root.get_entity_by_path("room/plant_pot")
    print(
        f"book1 at entity_path {book1.get_absolute_path()}, ranger at entity_path {ranger.get_absolute_path()}"
    )

    book1_pos = book1.getpos()
    print(f"book1_pos = {book1_pos}")


def main():
    sim_gen_graph()
    sim_run_flow()
    print("flow2 done")


if __name__ == "__main__":
    main()
