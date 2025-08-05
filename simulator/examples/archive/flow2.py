# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

import sys

sys.path.append(".")

import cv2
import numpy as np

from uapi.graph.entity import *
from uapi.log import logger, set_log_level
from uapi.specs.skill_specs import (
    EOS_TYPE_ImageFormat,
    EOS_TYPE_CameraType,
    EOS_TYPE_Image,
    EOS_TYPE_ImageMetadata,
)

root = None


def sim_gen_graph():
    global root
    root = create_root_room()
    logger.info(f"ID = {root.entity_id}, path = {root.get_absolute_path()}")

    room: Room = create_room_entity("room")
    root.add_child(room)
    logger.info(f"ID = {room.entity_id}, path = {room.get_absolute_path()}")

    ranger: Entity = create_controllable_entity("ranger")
    room.add_child(ranger)
    logger.info(f"ID = {ranger.entity_id}, path = {ranger.get_absolute_path()}")

    book1: Entity = create_generic_entity("book1")
    room.add_child(book1)
    logger.info(f"ID = {book1.entity_id}, path = {book1.get_absolute_path()}")

    book1.bind_skill("c_space_getpos", lambda: {"x": -2.2, "y": 1.8, "z": 0.1})

    def mock_image_capture():
        # Generate a colorful rainbow gradient image for testing
        width, height = 640, 480
        img = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                r = int(127.5 * (1 + np.sin(2 * np.pi * x / width)))
                g = int(127.5 * (1 + np.sin(2 * np.pi * y / height + 2)))
                b = int(
                    127.5 * (1 + np.sin(2 * np.pi * (x + y) / (width + height) + 4))
                )
                img[y, x] = (r, g, b)

        cv2.imshow("Rainbow Gradient", img)
        logger.info(f"Rainbow Gradient image generated, press any key to continue")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return EOS_TYPE_Image(
            image_raw=img.tobytes(),
            metadata=EOS_TYPE_ImageMetadata(
                width=width,
                height=height,
                format=EOS_TYPE_ImageFormat.JPEG,
                camera_type=EOS_TYPE_CameraType.RGB,
            ),
        )

    book1.bind_skill("c_image_capture", mock_image_capture)

    plant_pot: Entity = create_generic_entity("plant_pot")
    room.add_child(plant_pot)
    logger.info(f"ID = {plant_pot.entity_id}, path = {plant_pot.get_absolute_path()}")

    plant_pot.bind_skill("c_space_getpos", lambda: {"x": 2.0, "y": -2.0, "z": 0.2})


def sim_run_flow():
    book1 = root.get_entity_by_path("room/book1")
    ranger = root.get_entity_by_path("room/ranger")
    plant_pot = root.get_entity_by_path("room/plant_pot")
    print(
        f"book1 at entity_path {book1.get_absolute_path()}, ranger at entity_path {ranger.get_absolute_path()}"
    )

    book1_pos = book1.c_space_getpos()
    logger.info(f"book1_pos = {book1_pos}")

    # Call the c_image_capture skill and print the result
    book1_image = book1.c_image_capture()
    logger.info(f"book1_image = {book1_image}")


def main():
    set_log_level("DEBUG")
    sim_gen_graph()
    sim_run_flow()
    logger.info("flow2 done")


if __name__ == "__main__":
    main()
