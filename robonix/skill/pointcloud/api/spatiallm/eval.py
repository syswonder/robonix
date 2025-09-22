import os
import argparse
import math
import csv
import itertools
import logging
from typing import Dict, List
from collections import defaultdict
from dataclasses import dataclass

import torch
import pandas as pd
import numpy as np
from scipy.optimize import linear_sum_assignment
from shapely import Polygon
from bbox import BBox3D
from bbox.metrics import iou_3d
from terminaltables import AsciiTable

from spatiallm import Layout
from spatiallm.layout.entity import Wall, Door, Window, Bbox

log = logging.getLogger(__name__)

ZERO_TOLERANCE = 1e-6
LARGE_COST_VALUE = 1e6
LAYOUTS = ["wall", "door", "window"]
OBJECTS = [
    "curtain",
    "nightstand",
    "chandelier",
    "wardrobe",
    "bed",
    "sofa",
    "chair",
    "cabinet",
    "dining table",
    "plants",
    "tv cabinet",
    "coffee table",
    "side table",
    "air conditioner",
    "dresser",
    "stool",
    "refrigerator",
    "painting",
    "carpet",
    "tv",
]


@dataclass
class EvalTuple:
    tp: int
    num_pred: int
    num_gt: int

    @property
    def precision(self):
        return self.tp / self.num_pred if self.num_pred > 0 else 0

    @property
    def recall(self):
        return self.tp / self.num_gt if self.num_gt > 0 else 0

    @property
    def f1(self):
        return (
            (2 * self.precision * self.recall) / (self.precision + self.recall)
            if (self.precision + self.recall) > 0
            else 0
        )

    @property
    def masked(self):
        return self.num_pred == 0 and self.num_gt == 0


def calc_poly_iou(poly1, poly2):
    if poly1.intersects(poly2):
        inter_area = poly1.intersection(poly2).area
        union_area = poly1.union(poly2).area
        poly_iou = inter_area / union_area if union_area > 0 else 0
    else:
        poly_iou = 0
    return poly_iou


def read_label_mapping(
    label_path: str, label_from="spatiallm59", label_to="spatiallm23"
):
    assert os.path.isfile(label_path), f"Label mapping file {label_path} does not exist"
    mapping = dict()
    with open(label_path) as csvfile:
        reader = csv.DictReader(csvfile, delimiter="\t")
        for row in reader:
            label_from_value = row[label_from]
            label_to_value = row[label_to]
            if label_from_value == "" or label_to_value == "":
                continue
            mapping[label_from_value] = label_to_value
    return mapping


def assign_class_map(entities: List[Bbox], class_map=Dict[str, str]):
    res_entities = list()
    for entity in entities:
        mapping_to_class = class_map.get(entity.class_name.replace("_", " "))
        if mapping_to_class:
            entity.class_name = mapping_to_class
            res_entities.append(entity)
    return res_entities


def assign_minimum_scale(entities: List[Bbox], minimum_scale: float = 0.1):
    for entity in entities:
        entity.scale_x = max(entity.scale_x, minimum_scale)
        entity.scale_y = max(entity.scale_y, minimum_scale)
        entity.scale_z = max(entity.scale_z, minimum_scale)


def get_entity_class(entity):
    try:
        return entity.class_name
    except:
        return entity.entity_label


def get_BBox3D(entity: Bbox):
    return BBox3D(
        entity.position_x,
        entity.position_y,
        entity.position_z,
        entity.scale_x,
        entity.scale_y,
        entity.scale_z,
        euler_angles=[0, 0, entity.angle_z],
        is_center=True,
    )


def calc_bbox_tp(
    pred_entities: List[Bbox], gt_entities: List[Bbox], iou_threshold: float = 0.25
):
    num_pred = len(pred_entities)
    num_gt = len(gt_entities)
    if num_pred == 0 or num_gt == 0:
        return EvalTuple(0, num_pred, num_gt)

    iou_matrix = torch.as_tensor(
        [
            iou_3d(bbox_1, bbox_2)
            for bbox_1, bbox_2 in itertools.product(
                [get_BBox3D(entity) for entity in pred_entities],
                [get_BBox3D(entity) for entity in gt_entities],
            )
        ]
    ).resize(num_pred, num_gt)

    cost_matrix = torch.full((num_pred, num_gt), LARGE_COST_VALUE)
    cost_matrix[iou_matrix > iou_threshold] = -1

    indices = linear_sum_assignment(cost_matrix.numpy())

    tp_percent = iou_matrix[
        torch.as_tensor(indices[0], dtype=torch.int64),
        torch.as_tensor(indices[1], dtype=torch.int64),
    ]
    tp = torch.sum(tp_percent >= iou_threshold).item()

    return EvalTuple(tp, num_pred, num_gt)


def is_valid_wall(entity: Wall):
    wall_extent_x = max(max(entity.ax, entity.bx) - min(entity.ax, entity.bx), 0)
    wall_extent_y = max(max(entity.ay, entity.by) - min(entity.ay, entity.by), 0)
    return wall_extent_x > ZERO_TOLERANCE or wall_extent_y > ZERO_TOLERANCE


def is_valid_dw(entity: Door | Window, wall_id_lookup: Dict[int, Wall]):
    attach_wall = wall_id_lookup.get(entity.wall_id, None)
    if attach_wall is None:
        return False
    return is_valid_wall(attach_wall)


def get_corners(
    entity: Wall | Door | Window | Bbox, wall_id_lookup: Dict[int, Wall] = None
):
    if isinstance(entity, Wall):
        return np.array(
            [
                [entity.ax, entity.ay, entity.az],
                [entity.bx, entity.by, entity.bz],
                [entity.bx, entity.by, entity.bz + entity.height],
                [entity.ax, entity.ay, entity.az + entity.height],
            ]
        )
    elif isinstance(entity, (Door, Window)):
        attach_wall = wall_id_lookup.get(entity.wall_id, None)
        if attach_wall is None:
            log.error(f"{entity} attach wall is None")
            return

        wall_start = np.array([attach_wall.ax, attach_wall.ay])
        wall_end = np.array([attach_wall.bx, attach_wall.by])
        wall_length = np.linalg.norm(wall_end - wall_start)
        wall_xy_unit_vec = (wall_end - wall_start) / wall_length
        wall_xy_unit_vec = np.nan_to_num(wall_xy_unit_vec, nan=0)

        door_center = np.array(
            [entity.position_x, entity.position_y, entity.position_z]
        )
        offset = 0.5 * np.concatenate(
            [wall_xy_unit_vec * entity.width, np.array([entity.height])]
        )
        door_start_xyz = door_center - offset
        door_end_xyz = door_center + offset

        return np.array(
            [
                [door_start_xyz[0], door_start_xyz[1], door_start_xyz[2]],
                [door_end_xyz[0], door_end_xyz[1], door_start_xyz[2]],
                [door_end_xyz[0], door_end_xyz[1], door_end_xyz[2]],
                [door_start_xyz[0], door_start_xyz[1], door_end_xyz[2]],
            ]
        )
    elif isinstance(entity, Bbox):
        bbox_points = get_BBox3D(entity).p
        scale_key = ["scale_x", "scale_y", "scale_z"]
        match min(scale_key, key=lambda k: abs(getattr(entity, k))):
            case "scale_x":
                return np.array(
                    [
                        (bbox_points[0] + bbox_points[1]) / 2,
                        (bbox_points[2] + bbox_points[3]) / 2,
                        (bbox_points[6] + bbox_points[7]) / 2,
                        (bbox_points[4] + bbox_points[5]) / 2,
                    ]
                )
            case "scale_y":
                return np.array(
                    [
                        (bbox_points[0] + bbox_points[3]) / 2,
                        (bbox_points[4] + bbox_points[7]) / 2,
                        (bbox_points[5] + bbox_points[6]) / 2,
                        (bbox_points[1] + bbox_points[2]) / 2,
                    ]
                )
            case "scale_z":
                return np.array(
                    [
                        (bbox_points[0] + bbox_points[4]) / 2,
                        (bbox_points[1] + bbox_points[5]) / 2,
                        (bbox_points[2] + bbox_points[6]) / 2,
                        (bbox_points[3] + bbox_points[7]) / 2,
                    ]
                )
            case _:
                log.error(f"Unrecognized named attribute for {entity} in get_corners")
                return


def are_planes_parallel_and_close(
    corners_1: np.ndarray,
    corners_2: np.ndarray,
    parallel_tolerance: float,
    dist_tolerance: float,
):
    p1, p2, p3, _ = corners_1
    q1, q2, q3, _ = corners_2
    n1 = np.cross(np.subtract(p2, p1), np.subtract(p3, p1))
    n2 = np.cross(np.subtract(q2, q1), np.subtract(q3, q1))
    n1_length = np.linalg.norm(n1)
    n2_length = np.linalg.norm(n2)
    assert (
        n1_length * n2_length > ZERO_TOLERANCE
    ), f"Invalid plane corners, corners_1: {corners_1}, corners_2: {corners_2}"

    return (
        np.linalg.norm(np.cross(n1, n2)) / (n1_length * n2_length) < parallel_tolerance
        and np.dot(np.subtract(q1, p1), n1) / n1_length < dist_tolerance
    )


def calc_thin_bbox_iou_2d(
    corners_1: np.ndarray,
    corners_2: np.ndarray,
    parallel_tolerance: float,
    dist_tolerance: float,
):
    if are_planes_parallel_and_close(
        corners_1, corners_2, parallel_tolerance, dist_tolerance
    ):
        p1, p2, _, p4 = corners_2
        v1 = np.subtract(p2, p1)
        v2 = np.subtract(p4, p1)
        basis1 = v1 / np.linalg.norm(v1)
        basis1_orth = v2 - np.dot(v2, basis1) * basis1
        basis2 = basis1_orth / np.linalg.norm(basis1_orth)

        projected_corners_1 = [
            [
                np.dot(np.subtract(point, p1), basis1),
                np.dot(np.subtract(point, p1), basis2),
            ]
            for point in corners_1
        ]
        projected_corners_2 = [
            [
                np.dot(np.subtract(point, p1), basis1),
                np.dot(np.subtract(point, p1), basis2),
            ]
            for point in corners_2
        ]
        box1 = Polygon(projected_corners_1)
        box2 = Polygon(projected_corners_2)

        return calc_poly_iou(box1, box2)
    else:
        return 0


def calc_layout_tp(
    pred_entities: List[Wall | Door | Window],
    gt_entities: List[Wall | Door | Window],
    pred_wall_id_lookup: Dict[int, Wall],
    gt_wall_id_lookup: Dict[int, Wall],
    iou_threshold: float = 0.25,
    parallel_tolerance: float = math.sin(math.radians(5)),
    dist_tolerance: float = 0.2,
):
    num_pred = len(pred_entities)
    num_gt = len(gt_entities)
    if num_pred == 0 or num_gt == 0:
        return EvalTuple(0, num_pred, num_gt)

    iou_matrix = torch.as_tensor(
        [
            calc_thin_bbox_iou_2d(
                corners_1, corners_2, parallel_tolerance, dist_tolerance
            )
            for corners_1, corners_2 in itertools.product(
                [get_corners(entity, pred_wall_id_lookup) for entity in pred_entities],
                [get_corners(entity, gt_wall_id_lookup) for entity in gt_entities],
            )
        ]
    ).resize(num_pred, num_gt)

    cost_matrix = torch.full((num_pred, num_gt), LARGE_COST_VALUE)
    cost_matrix[iou_matrix > iou_threshold] = -1

    indices = linear_sum_assignment(cost_matrix.numpy())

    tp_percent = iou_matrix[
        torch.as_tensor(indices[0], dtype=torch.int64),
        torch.as_tensor(indices[1], dtype=torch.int64),
    ]
    tp = torch.sum(tp_percent >= iou_threshold).item()

    return EvalTuple(tp, num_pred, num_gt)


if __name__ == "__main__":
    parser = argparse.ArgumentParser("SpatialLM evaluation script")
    parser.add_argument(
        "--metadata",
        type=str,
        required=True,
        help="metadata CSV file with columns id, pcd, layout",
    )
    parser.add_argument(
        "--gt_dir",
        type=str,
        required=True,
        help="Path to the gt layout txt directory",
    )
    parser.add_argument(
        "--pred_dir",
        type=str,
        required=True,
        help="Path to the pred layout txt directory",
    )
    parser.add_argument(
        "--label_mapping",
        type=str,
        required=True,
        help="Path to the label mapping file",
    )
    parser.add_argument("--label_from", type=str, default="spatiallm59")
    parser.add_argument("--label_to", type=str, default="spatiallm20")
    args = parser.parse_args()

    df = pd.read_csv(args.metadata)
    scene_id_list = df["id"].tolist()
    class_map = read_label_mapping(args.label_mapping, args.label_from, args.label_to)

    classwise_eval_tuples_25: Dict[str, List[EvalTuple]] = defaultdict(list)
    classwise_eval_tuples_50: Dict[str, List[EvalTuple]] = defaultdict(list)
    for scene_id in scene_id_list:
        log.info(f"Evaluating scene {scene_id}")
        with open(os.path.join(args.pred_dir, f"{scene_id}.txt"), "r") as f:
            pred_layout = Layout(f.read())
        with open(os.path.join(args.gt_dir, f"{scene_id}.txt"), "r") as f:
            gt_layout = Layout(f.read())
        pred_layout.bboxes = assign_class_map(pred_layout.bboxes, class_map)
        gt_layout.bboxes = assign_class_map(gt_layout.bboxes, class_map)
        assign_minimum_scale(pred_layout.bboxes, minimum_scale=0.1)
        assign_minimum_scale(gt_layout.bboxes, minimum_scale=0.1)

        # Layout, F1
        pred_wall_id_lookup = {w.id: w for w in pred_layout.walls}
        gt_wall_id_lookup = {w.id: w for w in gt_layout.walls}

        pred_layout_instances = list(
            filter(lambda e: is_valid_wall(e), pred_layout.walls)
        ) + list(
            filter(
                lambda e: is_valid_dw(e, pred_wall_id_lookup),
                pred_layout.doors + pred_layout.windows,
            )
        )
        gt_layout_instances = list(
            filter(lambda e: is_valid_wall(e), gt_layout.walls)
        ) + list(
            filter(
                lambda e: is_valid_dw(e, gt_wall_id_lookup),
                gt_layout.doors + gt_layout.windows,
            )
        )
        for class_name in LAYOUTS:
            classwise_eval_tuples_25[class_name].append(
                calc_layout_tp(
                    pred_entities=[
                        b
                        for b in pred_layout_instances
                        if get_entity_class(b) == class_name
                    ],
                    gt_entities=[
                        b
                        for b in gt_layout_instances
                        if get_entity_class(b) == class_name
                    ],
                    pred_wall_id_lookup=pred_wall_id_lookup,
                    gt_wall_id_lookup=gt_wall_id_lookup,
                    iou_threshold=0.25,
                )
            )

            classwise_eval_tuples_50[class_name].append(
                calc_layout_tp(
                    pred_entities=[
                        b
                        for b in pred_layout_instances
                        if get_entity_class(b) == class_name
                    ],
                    gt_entities=[
                        b
                        for b in gt_layout_instances
                        if get_entity_class(b) == class_name
                    ],
                    pred_wall_id_lookup=pred_wall_id_lookup,
                    gt_wall_id_lookup=gt_wall_id_lookup,
                    iou_threshold=0.50,
                )
            )

        # Normal Objects, F1
        pred_normal_objects = [b for b in pred_layout.bboxes if b.class_name in OBJECTS]
        gt_normal_objects = [b for b in gt_layout.bboxes if b.class_name in OBJECTS]
        for class_name in OBJECTS:
            classwise_eval_tuples_25[class_name].append(
                calc_bbox_tp(
                    pred_entities=[
                        b
                        for b in pred_normal_objects
                        if get_entity_class(b) == class_name
                    ],
                    gt_entities=[
                        b
                        for b in gt_normal_objects
                        if get_entity_class(b) == class_name
                    ],
                    iou_threshold=0.25,
                )
            )

            classwise_eval_tuples_50[class_name].append(
                calc_bbox_tp(
                    pred_entities=[
                        b
                        for b in pred_normal_objects
                        if get_entity_class(b) == class_name
                    ],
                    gt_entities=[
                        b
                        for b in gt_normal_objects
                        if get_entity_class(b) == class_name
                    ],
                    iou_threshold=0.50,
                )
            )

    headers = ["Layouts", "F1 @.25 IoU", "F1 @.50 IoU"]
    table_data = [headers]
    for class_name in LAYOUTS:
        tuples = classwise_eval_tuples_25[class_name]
        f1_25 = np.ma.masked_where(
            [t.masked for t in tuples], [t.f1 for t in tuples]
        ).mean()

        tuples = classwise_eval_tuples_50[class_name]
        f1_50 = np.ma.masked_where(
            [t.masked for t in tuples], [t.f1 for t in tuples]
        ).mean()

        table_data.append([class_name, f1_25, f1_50])
    print("\n" + AsciiTable(table_data).table)

    headers = ["Objects", "F1 @.25 IoU", "F1 @.50 IoU"]
    table_data = [headers]
    for class_name in OBJECTS:
        tuples = classwise_eval_tuples_25[class_name]
        f1_25 = np.ma.masked_where(
            [t.masked for t in tuples], [t.f1 for t in tuples]
        ).mean()

        tuples = classwise_eval_tuples_50[class_name]
        f1_50 = np.ma.masked_where(
            [t.masked for t in tuples], [t.f1 for t in tuples]
        ).mean()

        table_data.append([class_name, f1_25, f1_50])
    print("\n" + AsciiTable(table_data).table)
