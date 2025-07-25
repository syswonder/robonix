# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from entity import *
import time

root: Room = create_root_room()
print(f"ID = {root.entity_id}, path = {root.get_absolute_path()}")

building = create_room_entity("building")
root.add_child(building)
print(f"ID = {building.entity_id}, path = {building.get_absolute_path()}")

room1 = create_room_entity("room1")
building.add_child(room1)
print(f"ID = {room1.entity_id}, path = {room1.get_absolute_path()}")

apple = create_generic_entity("apple")
room1.add_child(apple)
print(f"ID = {apple.entity_id}, path = {apple.get_absolute_path()}")

agilex_ranger = create_computing_entity("agilex_ranger")
room1.add_child(agilex_ranger)
print(f"ID = {agilex_ranger.entity_id}, path = {agilex_ranger.get_absolute_path()}")

while True:
    print(".")
    time.sleep(1)
