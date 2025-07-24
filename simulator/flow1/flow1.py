# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import sys

sys.path.append(".")
sys.path.append("./simulator/genesis")

from manager.graph.entity import *
from driver.sim_genesis_ranger.driver import move_to_point
from driver.sim_genesis_ranger.driver import get_pose

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

    # Bind getpos primitive to book1, always returns fixed position
    book1.bind_primitive("getpos", lambda: {"x": -2.2, "y": 1.8, "z": 0.1})

    plant_pot: Entity = create_generic_entity("plant_pot")
    room.add_child(plant_pot)
    print(f"ID = {plant_pot.entity_id}, path = {plant_pot.get_absolute_path()}")

    plant_pot.bind_primitive("getpos", lambda: {"x": 2.0, "y": -2.0, "z": 0.2})

    def move_impl(x, y, z):
        move_to_point(x, y)  # THIS IS A FUNCTION FROM DRIVER !
        return {"success": True}

    ranger.bind_primitive("move", move_impl)

    def __get_pose_impl():
        x, y, z = get_pose()  # THIS IS A FUNCTION FROM DRIVER !
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert isinstance(z, float)
        return {"x": x, "y": y, "z": z}

    ranger.bind_primitive("getpos", __get_pose_impl)


def sim_run_flow():
    book1 = root.get_entity_by_path("room/book1")
    ranger = root.get_entity_by_path("room/ranger")
    plant_pot = root.get_entity_by_path("room/plant_pot")
    print(
        f"book1 at entity_path {book1.get_absolute_path()}, ranger at entity_path {ranger.get_absolute_path()}"
    )

    book1_pos = book1.getpos()
    print(f"book1_pos = {book1_pos}")

    # ranger.move(x=book1_pos["x"], y=book1_pos["y"] + 0.5, z=book1_pos["z"])
    # ranger.move(x=0, y=0, z=0)
    ranger.move(x="hello", y=0.2, z={"hello1": "world2"})

    print(f"finished moving to book1, ranger now at {ranger.getpos()}")

    virtual_waypoint1 = create_generic_entity("virtual_waypoint1")
    virtual_waypoint1.bind_primitive("getpos", lambda: {"x": 1.0, "y": 0.0, "z": 0.0})
    root.get_entity_by_path("room").add_child(virtual_waypoint1)
    print(f"virtual_waypoint1 = {virtual_waypoint1.get_absolute_path()}")

    virtual_waypoint1_pos = virtual_waypoint1.getpos()

    ranger.move(
        x=virtual_waypoint1_pos["x"],
        y=virtual_waypoint1_pos["y"],
        z=virtual_waypoint1_pos["z"],
    )

    print(f"finished moving to waypoint, ranger now at {ranger.getpos()}")

    plant_pot_pos = plant_pot.getpos()
    ranger.move(
        x=plant_pot_pos["x"] + 0.5,
        y=plant_pot_pos["y"],
        z=plant_pot_pos["z"],
    )

    print(f"finished moving to plant_pot, ranger now at {ranger.getpos()}")


def main():
    sim_gen_graph()
    sim_run_flow()
    print("flow1 done")


if __name__ == "__main__":
    main()
