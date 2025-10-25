import os
import argparse
import json
from glob import glob

import pandas as pd
from tqdm import tqdm

from spatiallm.tuner.data import (
    LAYOUT_S_PLACEHOLDER,
    LAYOUT_E_PLACEHOLDER,
    POINT_CLOUD_PLACEHOLDER,
)
from spatiallm.layout.layout import Layout

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--dataset_dir",
        type=str,
        required=True,
        help="Path to the input point cloud directory",
    )
    parser.add_argument(
        "-s",
        "--split_csv",
        type=str,
        required=True,
        help="Path to the split csv file",
    )
    parser.add_argument(
        "--code_template_file",
        type=str,
        default="code_template.txt",
    )
    parser.add_argument(
        "-n",
        "--dataset_name",
        type=str,
        required=True,
        help="Name of the dataset",
    )
    args = parser.parse_args()

    pcd_dir = os.path.join(args.dataset_dir, "pcd")
    layout_dir = os.path.join(args.dataset_dir, "layout")

    pcd_files = glob(os.path.join(pcd_dir, "*.ply"))
    pcd_scene_ids = [os.path.basename(pcd_file).split(".")[0] for pcd_file in pcd_files]
    layout_files = glob(os.path.join(layout_dir, "*.txt"))
    layout_scene_ids = [
        os.path.basename(layout_file).split(".")[0] for layout_file in layout_files
    ]

    # find the common filenames between pcd_filenames and layout_filenames
    scene_ids = set(pcd_scene_ids) & set(layout_scene_ids)
    scene_ids = list(scene_ids)

    df = pd.read_csv(args.split_csv, dtype=str)
    df.set_index("id", inplace=True)

    print(f"Creating dataset with {len(scene_ids)} scenes...")

    with open(args.code_template_file, "r") as f:
        code_template = f.read()

    dataset = {
        "train": [],
        "val": [],
    }
    for si, scene_id in enumerate(tqdm(scene_ids)):
        try:
            if scene_id not in df.index:
                continue
            split = df.loc[scene_id, "split"]

            # load txt file
            with open(os.path.join(layout_dir, f"{scene_id}.txt"), "r") as f:
                layout_content = f.read()

            layout = Layout(layout_content)
            language_string = layout.to_language_string()

            conversation_data = {
                "conversations": [
                    {
                        "from": "human",
                        "value": f"{POINT_CLOUD_PLACEHOLDER}Detect boxes. The reference code is as followed: {code_template}",
                    },
                    {
                        "from": "gpt",
                        "value": f"{LAYOUT_S_PLACEHOLDER}{language_string}{LAYOUT_E_PLACEHOLDER}",
                    },
                ],
                "point_clouds": [
                    os.path.join(os.path.basename(pcd_dir), f"{scene_id}.ply"),
                ],
            }
            dataset[split].append(conversation_data)
        except Exception as e:
            print(f"Error processing scene {scene_id}: {e}")
            continue

    # save train set to train.json
    print(f"Saving train set with {len(dataset['train'])} samples...")
    with open(
        os.path.join(args.dataset_dir, f"{args.dataset_name}_train.json"), "w"
    ) as f:
        json.dump(dataset["train"], f, indent=2)

    # save val set to val.json
    print(f"Saving val set with {len(dataset['val'])} samples...")
    with open(
        os.path.join(args.dataset_dir, f"{args.dataset_name}_val.json"), "w"
    ) as f:
        json.dump(dataset["val"], f, indent=2)

    # update dataset_info.json
    dataset_info = {
        f"{args.dataset_name}_train": {
            "file_name": f"{args.dataset_name}_train.json",
            "formatting": "sharegpt",
            "columns": {
                "messages": "conversations",
                "point_clouds": "point_clouds",
            },
        },
        f"{args.dataset_name}_val": {
            "file_name": f"{args.dataset_name}_val.json",
            "formatting": "sharegpt",
            "columns": {
                "messages": "conversations",
                "point_clouds": "point_clouds",
            },
        },
    }

    if not os.path.exists(os.path.join(args.dataset_dir, "dataset_info.json")):
        with open(os.path.join(args.dataset_dir, "dataset_info.json"), "w") as f:
            json.dump(dataset_info, f, indent=2)
    else:
        with open(os.path.join(args.dataset_dir, "dataset_info.json"), "r") as f:
            original_dataset_info = json.load(f)
        original_dataset_info.update(dataset_info)
        with open(os.path.join(args.dataset_dir, "dataset_info.json"), "w") as f:
            json.dump(original_dataset_info, f, indent=2)
