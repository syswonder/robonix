import os
from copy import deepcopy
from typing import TYPE_CHECKING, Dict, List, Union, Sequence, Optional, Tuple

import torch
import numpy as np
from scipy.spatial.transform import Rotation as R

from spatiallm.layout.layout import Layout
from spatiallm.layout.entity import NORMALIZATION_PRESET
from spatiallm.pcd import load_o3d_pcd, get_points_and_colors
from spatiallm.pcd.transform import Compose

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from transformers import PreTrainedTokenizer

    PointCloudInput = Union[str, dict, NDArray]

LAYOUT_S_PLACEHOLDER = os.environ.get("LAYOUT_S_PLACEHOLDER", "<|layout_s|>")
LAYOUT_E_PLACEHOLDER = os.environ.get("LAYOUT_E_PLACEHOLDER", "<|layout_e|>")
POINT_S_TOKEN = os.environ.get("POINT_S_TOKEN", "<|point_start|>")
POINT_E_TOKEN = os.environ.get("POINT_E_TOKEN", "<|point_end|>")
POINT_CLOUD_PLACEHOLDER = os.environ.get("POINT_CLOUD_PLACEHOLDER", "<point_cloud>")


class SpatialLMPlugin:
    def __init__(
        self,
        point_token: str = "<|point_pad|>",
        num_bins: int = 1280,
        do_augmentation: bool = False,
        random_rotation: bool = False,
    ):
        self.point_token = point_token

        global_extent = NORMALIZATION_PRESET["world"]
        self.num_bins = num_bins
        self.grid_size = (global_extent[1] - global_extent[0]) / self.num_bins
        self.do_augmentation = do_augmentation
        self.random_rotation = random_rotation
        self.augmentation = Compose(
            [
                dict(type="RandomColorGrayScale", p=0.05),
                dict(type="ChromaticAutoContrast", p=0.2, blend_factor=None),
                dict(type="ChromaticTranslation", p=0.75, ratio=0.1),
                dict(type="ChromaticJitter", p=0.8, std=0.05),
                dict(type="HueSaturationTranslation", hue_max=0.2, saturation_max=0.2),
                dict(type="RandomColorDrop", p=0.1, color_augment=0.0),
                dict(type="RandomJitter", sigma=0.025, clip=0.05, ratio=0.8, p=0.9),
                dict(type="RandomJitter", sigma=0.2, clip=0.2, ratio=0.05, p=0.85),
                dict(type="RandomJitter", sigma=0.4, clip=1.0, ratio=0.001, p=0.75),
                dict(type="RandomJitter", sigma=0.5, clip=4.0, ratio=0.0005, p=0.7),
                dict(
                    type="ElasticDistortion",
                    distortion_params=[[0.2, 0.4], [0.8, 1.6]],
                    p=[0.85, 0.5],
                ),
            ]
        )

        self.transform = Compose(
            [
                dict(type="PositiveShift"),
                dict(type="NormalizeColor"),
                dict(
                    type="GridSample",
                    grid_size=self.grid_size,
                    hash_type="fnv",
                    mode="train",
                    keys=("coord", "color"),
                    return_grid_coord=True,
                    max_grid_coord=self.num_bins,
                ),
            ]
        )

    def _preprocess_point_cloud(self, point_cloud: dict) -> np.ndarray:
        r"""
        Pre-processes a single point cloud.
        """
        point_cloud = self.transform(point_cloud)
        coord = point_cloud["grid_coord"]
        xyz = point_cloud["coord"]
        color = point_cloud["color"]
        assert len(coord) == len(xyz) == len(color)
        return np.concatenate([coord, xyz, color], axis=1)

    def _regularize_point_clouds(
        self, point_clouds: Sequence["PointCloudInput"], **kwargs
    ) -> torch.Tensor:
        points_list = []
        max_len = 0
        for point_cloud in point_clouds:
            if not isinstance(point_cloud, dict):
                raise ValueError(
                    "Point cloud input must be a dictionary with 'name' and 'coord' keys."
                )
            point_feats = self._preprocess_point_cloud(point_cloud, **kwargs)
            max_len = max(max_len, len(point_feats))
            points_list.append(point_feats)

        for i in range(len(points_list)):
            points_list[i] = np.pad(
                points_list[i],
                ((0, max_len - len(points_list[i])), (0, 0)),
                mode="constant",
                constant_values=np.nan,
            )

        # convert list of point clouds to batch with shape (batch_size, max_len, 3)
        return torch.as_tensor(np.stack(points_list, axis=0))

    def _get_mm_inputs(
        self,
        batched_messages: Sequence[Dict[str, str]],
        point_clouds: Sequence["PointCloudInput"],
    ) -> dict:
        input_dict = {"point_clouds": None}  # default key

        point_clouds_data = []
        transformations = []
        for pcd_path in point_clouds:
            pcd = load_o3d_pcd(pcd_path)
            points, colors = get_points_and_colors(pcd)

            if self.do_augmentation:
                data_aug = {"name": "pcd", "coord": points, "color": colors}
                data_aug = self.augmentation(data_aug)
                points = data_aug["coord"]
                colors = data_aug["color"]

            # randomly apply scale and rotation transformation to the point cloud
            if self.random_rotation:
                angle_z = np.random.random() * 2 * np.pi
            else:
                angle_z = np.random.choice(np.array([0, 0.5, 1.0, 1.5]) * np.pi)

            scaling = np.random.uniform(0.75, 1.25)
            rotmat = R.from_rotvec(np.array([0, 0, angle_z])).as_matrix()
            min_bound = points.min(axis=0)
            max_bound = points.max(axis=0)
            center_pt = (min_bound + max_bound) / 2
            scaled_points = (points - center_pt) * scaling
            transformed_points = (rotmat @ scaled_points.T).T + center_pt
            # store transformation parameters for sync the augmentation to the layout
            transformations.append(
                {
                    "angle_z": angle_z,
                    "center_pt": center_pt,
                    "scaling": scaling,
                    "min_bound": np.min(transformed_points, axis=0),
                    "transformed_points": transformed_points,
                }
            )

            point_cloud = {"name": "pcd", "coord": transformed_points, "color": colors}
            point_clouds_data.append(point_cloud)

        # Here we assume each conversation has exactly one point cloud
        assert len(batched_messages) == len(point_clouds_data)
        processed_messages = []
        for mi, messages in enumerate(batched_messages):
            processed_messages.append(
                self.process_messages(messages, [transformations[mi]])
            )

        if len(processed_messages) != 0:
            input_dict["messages"] = processed_messages
        if len(point_clouds_data) != 0:
            # convert point clouds to batched tensors with shape (batch_size, max_len, 9)
            input_dict["point_clouds"] = self._regularize_point_clouds(
                point_clouds_data
            )
        return input_dict

    def _validate_input(
        self,
        point_clouds: Sequence["PointCloudInput"],
    ) -> None:
        r"""
        Validates if this model accepts the input modalities.
        """
        if len(point_clouds) != 0 and self.point_token is None:
            raise ValueError(
                "This model does not support point cloud input. Please check whether the correct `template` is used."
            )

    def process_token_ids(
        self,
        input_ids: List[int],
        labels: Optional[List[int]],
        point_clouds: Sequence["PointCloudInput"],
        tokenizer: "PreTrainedTokenizer",
    ) -> Tuple[List[int], Optional[List[int]]]:
        self._validate_input(point_clouds)
        return input_ids, labels

    def process_messages(
        self,
        messages: Sequence[Dict[str, str]],
        transformations: Sequence[dict],
    ) -> List[Dict[str, str]]:
        r"""
        Pre-processes input messages to sync the transformation between point cloud and layout.
        """
        self._validate_input(transformations)
        messages = deepcopy(messages)
        num_point_tokens = 0

        for message in messages:
            content = message["content"]
            if LAYOUT_S_PLACEHOLDER in content and LAYOUT_E_PLACEHOLDER in content:
                transformation = transformations[num_point_tokens - 1]
                min_bound = transformation["min_bound"]
                center_pt = transformation["center_pt"]
                scaling = transformation["scaling"]
                transformed_points = transformation["transformed_points"]
                layout_start_pos = content.index(LAYOUT_S_PLACEHOLDER)
                layout_end_pos = content.index(LAYOUT_E_PLACEHOLDER)
                layout_content = content[
                    layout_start_pos + len(LAYOUT_S_PLACEHOLDER) : layout_end_pos
                ]
                # parse layout_content
                layout = Layout(layout_content)
                # transformation augmentation
                layout.translate(-center_pt)
                layout.scale(scaling)
                layout.rotate(transformation["angle_z"])
                layout.translate(center_pt)
                layout.filter_empty_bboxes(transformed_points, num_points=100)
                layout.reorder_entities()
                layout.translate(-min_bound)
                layout.normalize_and_discretize(self.num_bins)
                new_layout_content = layout.to_language_string()
                content = content.replace(
                    f"{LAYOUT_S_PLACEHOLDER}{layout_content}{LAYOUT_E_PLACEHOLDER}",
                    new_layout_content,
                )
                message["content"] = content

            if POINT_CLOUD_PLACEHOLDER in content:
                content = content.replace(
                    POINT_CLOUD_PLACEHOLDER,
                    f"{POINT_S_TOKEN}{self.point_token}{POINT_E_TOKEN}",
                    1,
                )
                num_point_tokens += 1
                message["content"] = content

        if len(transformations) != num_point_tokens:
            raise ValueError(
                f"The number of point clouds does not match the number of {POINT_CLOUD_PLACEHOLDER} tokens."
            )
        return messages

    def get_mm_inputs(
        self,
        point_clouds: Sequence["PointCloudInput"],
        batch_prompts: Sequence[List[int]],
    ) -> Dict[str, Union[List[dict]]]:
        r"""
        Builds batched multimodal inputs for VLMs.

        Arguments:
            point_clouds: a list of point cloud inputs, shape (num_point_clouds,)
            pointlens: number of point clouds in each sample, shape (batch_size,)
            batch_ids: token ids of input samples, shape (batch_size, seq_len)
            processor: a processor for pre-processing images and videos
        """
        self._validate_input(point_clouds)
        return self._get_mm_inputs(batch_prompts, point_clouds)


def get_mm_plugin(
    point_token: str = "<|point_pad|>",
    **kwargs,
) -> "SpatialLMPlugin":
    return SpatialLMPlugin(point_token, **kwargs)
