import numpy as np
from scipy.spatial.transform import Rotation as R
from spatiallm.layout.entity import Wall, Door, Window, Bbox, NORMALIZATION_PRESET


class Layout:
    def __init__(self, s: str = None):
        self.walls = []
        self.doors = []
        self.windows = []
        self.bboxes = []

        if s:
            self.from_str(s)

    @staticmethod
    def get_grid_size(num_bins):
        world_min, world_max = NORMALIZATION_PRESET["world"]
        return (world_max - world_min) / num_bins

    def from_str(self, s: str):
        s = s.lstrip("\n")
        lines = s.split("\n")
        # wall lookup table
        existing_walls = []
        for line in lines:
            try:
                label = line.split("=")[0]
                entity_id = int(label.split("_")[1])
                entity_label = label.split("_")[0]

                # extract params
                start_pos = line.find("(")
                end_pos = line.find(")")
                params = line[start_pos + 1 : end_pos].split(",")

                if entity_label == Wall.entity_label:
                    wall_args = [
                        "ax",
                        "ay",
                        "az",
                        "bx",
                        "by",
                        "bz",
                        "height",
                        "thickness",
                    ]
                    wall_params = dict(zip(wall_args, params[0:8]))
                    entity = Wall(id=entity_id, **wall_params)
                    existing_walls.append(entity_id)
                    self.walls.append(entity)
                elif entity_label == Door.entity_label:
                    wall_id = int(params[0].split("_")[1])
                    if wall_id not in existing_walls:
                        continue

                    door_args = [
                        "position_x",
                        "position_y",
                        "position_z",
                        "width",
                        "height",
                    ]
                    door_params = dict(zip(door_args, params[1:6]))
                    entity = Door(
                        id=entity_id,
                        wall_id=wall_id,
                        **door_params,
                    )
                    self.doors.append(entity)
                elif entity_label == Window.entity_label:
                    wall_id = int(params[0].split("_")[1])
                    if wall_id not in existing_walls:
                        continue

                    window_args = [
                        "position_x",
                        "position_y",
                        "position_z",
                        "width",
                        "height",
                    ]
                    window_params = dict(zip(window_args, params[1:6]))
                    entity = Window(
                        id=entity_id,
                        wall_id=wall_id,
                        **window_params,
                    )
                    self.windows.append(entity)
                elif entity_label == Bbox.entity_label:
                    class_name = params[0]
                    bbox_args = [
                        "position_x",
                        "position_y",
                        "position_z",
                        "angle_z",
                        "scale_x",
                        "scale_y",
                        "scale_z",
                    ]
                    bbox_params = dict(zip(bbox_args, params[1:8]))
                    entity = Bbox(
                        id=entity_id,
                        class_name=class_name,
                        **bbox_params,
                    )
                    self.bboxes.append(entity)
            except Exception as e:
                continue

    def to_boxes(self):
        boxes = []
        lookup = {}
        for wall in self.walls:
            # assume the walls has a thickness of 0.0 for now
            thickness = 0.0
            corner_a = np.array([wall.ax, wall.ay, wall.az])
            corner_b = np.array([wall.bx, wall.by, wall.bz])
            length = np.linalg.norm(corner_a - corner_b)
            direction = corner_b - corner_a
            angle = np.arctan2(direction[1], direction[0])
            lookup[wall.id] = {"wall": wall, "angle": angle}

            center = (corner_a + corner_b) * 0.5 + np.array([0, 0, 0.5 * wall.height])
            scale = np.array([length, thickness, wall.height])
            rotation = R.from_rotvec([0, 0, angle]).as_matrix()
            box = {
                "id": wall.id,
                "class": Wall.entity_label,
                "label": Wall.entity_label,
                "center": center,
                "rotation": rotation,
                "scale": scale,
            }
            boxes.append(box)

        for fixture in self.doors + self.windows:
            wall_id = fixture.wall_id
            wall_info = lookup.get(wall_id, None)
            if wall_info is None:
                continue

            wall = wall_info["wall"]
            angle = wall_info["angle"]
            thickness = wall.thickness

            center = np.array(
                [fixture.position_x, fixture.position_y, fixture.position_z]
            )
            scale = np.array([fixture.width, thickness, fixture.height])
            rotation = R.from_rotvec([0, 0, angle]).as_matrix()
            class_prefix = 1000 if fixture.entity_label == Door.entity_label else 2000
            box = {
                "id": fixture.id + class_prefix,
                "class": fixture.entity_label,
                "label": fixture.entity_label,
                "center": center,
                "rotation": rotation,
                "scale": scale,
            }
            boxes.append(box)

        for bbox in self.bboxes:
            center = np.array([bbox.position_x, bbox.position_y, bbox.position_z])
            scale = np.array([bbox.scale_x, bbox.scale_y, bbox.scale_z])
            rotation = R.from_rotvec([0, 0, bbox.angle_z]).as_matrix()
            class_name = bbox.class_name
            box = {
                "id": bbox.id + 3000,
                "class": Bbox.entity_label,
                "label": class_name,
                "center": center,
                "rotation": rotation,
                "scale": scale,
            }
            boxes.append(box)

        return boxes

    def get_entities(self):
        return self.walls + self.doors + self.windows + self.bboxes

    def normalize_and_discretize(self, num_bins):
        for entity in self.get_entities():
            entity.normalize_and_discretize(num_bins)

    def undiscretize_and_unnormalize(self, num_bins):
        for entity in self.get_entities():
            entity.undiscretize_and_unnormalize(num_bins)

    def translate(self, translation: np.ndarray):
        for entity in self.get_entities():
            entity.translate(translation)

    def rotate(self, angle: float):
        for entity in self.get_entities():
            entity.rotate(angle)

    def scale(self, scale: float):
        for entity in self.get_entities():
            entity.scale(scale)

    def to_language_string(self):
        entity_strings = []
        for entity in self.get_entities():
            entity_strings.append(entity.to_language_string())
        return "\n".join(entity_strings)

    def filter_empty_bboxes(self, points, num_points=100, margin=0.15):
        filtered_bboxes = []
        for bbox in self.bboxes:
            box_center = np.array([bbox.position_x, bbox.position_y, bbox.position_z])
            box_size = np.array([bbox.scale_x, bbox.scale_y, bbox.scale_z])
            rot_mat = R.from_rotvec(np.array([0, 0, bbox.angle_z])).as_matrix()

            translated_points = points - np.array(box_center)
            rotated_points = translated_points @ rot_mat.T
            box_min = -box_size / 2 - margin
            box_max = box_size / 2 + margin
            points_mask = np.all(rotated_points >= box_min, axis=1) & np.all(
                rotated_points <= box_max, axis=1
            )
            if np.sum(points_mask) > num_points:
                filtered_bboxes.append(bbox)

        self.bboxes = filtered_bboxes
        return filtered_bboxes

    @staticmethod
    def sort_entities(entities, wall_lookup=None):
        if not entities:
            return []
        sort_keys = [entity.sort_key() for entity in entities]
        sorted_idx = np.lexsort(np.stack(sort_keys).T)
        sorted_entities = [entities[i] for i in sorted_idx]
        for i, entity in enumerate(sorted_entities):
            entity.id = i
            if wall_lookup is not None and hasattr(entity, "wall_id"):
                entity.wall_id = wall_lookup[entity.wall_id]
        return sorted_entities

    def reorder_entities(
        self,
        filter_small_objects=True,
        filter_unknown=True,
        size_threshold=0.15,
    ):
        bboxes = []
        # filter invalid bboxes
        for bbox in self.bboxes:
            if filter_small_objects and (
                bbox.scale_x < size_threshold
                and bbox.scale_y < size_threshold
                and bbox.scale_z < size_threshold
            ):
                continue
            if filter_unknown and (not bbox.class_name or bbox.class_name == "unknown"):
                continue
            bboxes.append(bbox)

        wall_lookup = {}
        sorted_walls = []
        if len(self.walls) > 0:
            sort_keys = []
            for wi, wall in enumerate(self.walls):
                sort_keys.append(wall.sort_key())
            sorted_idx = np.lexsort(np.stack(sort_keys).T)
            sorted_walls = [self.walls[i] for i in sorted_idx]
            for wi, wall in enumerate(sorted_walls):
                wall_lookup[wall.id] = wi
                wall.id = wi

        sorted_doors = self.sort_entities(self.doors, wall_lookup)
        sorted_windows = self.sort_entities(self.windows, wall_lookup)
        sorted_bboxes = self.sort_entities(bboxes)

        self.walls = sorted_walls
        self.doors = sorted_doors
        self.windows = sorted_windows
        self.bboxes = sorted_bboxes
