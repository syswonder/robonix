import os
import sys
import json
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, Optional, TypedDict, Union

import numpy as np
from datasets import (
    Dataset,
    DatasetDict,
    load_dataset,
    load_from_disk,
    concatenate_datasets,
)
from huggingface_hub import hf_hub_download

from ..framework import logging
from ..framework.loader import use_modelscope
from ..framework.utils import check_version
from .converter import align_dataset

if TYPE_CHECKING:
    from datasets import IterableDataset
    from transformers import Seq2SeqTrainingArguments

    from ..hparams import DataArguments, ModelArguments


logger = logging.get_logger(__name__)

DATA_CONFIG = "dataset_info.json"
FILEEXT2TYPE = {
    "arrow": "arrow",
    "csv": "csv",
    "json": "json",
    "jsonl": "json",
    "parquet": "parquet",
    "txt": "text",
}


class DatasetModule(TypedDict):
    train_dataset: Optional[Union["Dataset", "IterableDataset"]]
    eval_dataset: Optional[Union["Dataset", "IterableDataset", dict[str, "Dataset"]]]


@dataclass
class DatasetAttr:
    r"""Dataset attributes."""

    # basic configs
    load_from: Literal["hf_hub", "ms_hub", "file"]
    dataset_name: str
    formatting: Literal["alpaca", "sharegpt"] = "alpaca"
    # extra configs
    subset: Optional[str] = None
    split: str = "train"
    folder: Optional[str] = None
    num_samples: Optional[int] = None
    # common columns
    system: Optional[str] = None
    point_clouds: Optional[str] = None
    # alpaca columns
    prompt: Optional[str] = "instruction"
    query: Optional[str] = "input"
    response: Optional[str] = "output"
    history: Optional[str] = None
    # sharegpt columns
    messages: Optional[str] = "conversations"
    # sharegpt tags
    role_tag: Optional[str] = "from"
    content_tag: Optional[str] = "value"
    user_tag: Optional[str] = "human"
    assistant_tag: Optional[str] = "gpt"
    system_tag: Optional[str] = "system"

    def __repr__(self) -> str:
        return self.dataset_name

    def set_attr(
        self, key: str, obj: dict[str, Any], default: Optional[Any] = None
    ) -> None:
        setattr(self, key, obj.get(key, default))

    def join(self, attr: dict[str, Any]) -> None:
        self.set_attr("formatting", attr, default="alpaca")
        self.set_attr("subset", attr)
        self.set_attr("split", attr, default="train")
        self.set_attr("folder", attr)
        self.set_attr("num_samples", attr)

        if "columns" in attr:
            column_names = [
                "prompt",
                "query",
                "response",
                "history",
                "messages",
                "system",
                "point_clouds",
            ]
            for column_name in column_names:
                self.set_attr(column_name, attr["columns"])

        if "tags" in attr:
            tag_names = ["role_tag", "content_tag"]
            tag_names += ["user_tag", "assistant_tag", "system_tag"]
            for tag in tag_names:
                self.set_attr(tag, attr["tags"])


def has_preprocessed_data(path: "os.PathLike") -> bool:
    r"""Check if the path has a preprocessed dataset."""
    return os.path.isdir(path) and len(os.listdir(path)) > 0


def get_dataset_module(dataset: Union["Dataset", "DatasetDict"]) -> "DatasetModule":
    r"""Convert dataset or dataset dict to dataset module."""
    dataset_module: DatasetModule = {}
    if isinstance(dataset, DatasetDict):  # dataset dict
        if "train" in dataset:
            dataset_module["train_dataset"] = dataset["train"]

        if "validation" in dataset:
            dataset_module["eval_dataset"] = dataset["validation"]
        else:
            eval_dataset = {}
            for key in dataset.keys():
                if key.startswith("validation_"):
                    eval_dataset[key[len("validation_") :]] = dataset[key]

            if len(eval_dataset):
                dataset_module["eval_dataset"] = eval_dataset

    else:  # single dataset
        dataset_module["train_dataset"] = dataset

    return dataset_module


def split_dataset(
    dataset: Optional[Union["Dataset", "IterableDataset"]],
    eval_dataset: Optional[Union["Dataset", "IterableDataset", dict[str, "Dataset"]]],
    data_args: "DataArguments",
    seed: int,
) -> "DatasetDict":
    r"""Split the dataset and returns a dataset dict containing train set and validation set.

    Support both map dataset and iterable dataset.
    """
    if eval_dataset is not None and data_args.val_size > 1e-6:
        raise ValueError("Cannot specify `val_size` if `eval_dataset` is not None.")

    dataset_dict = {}
    if dataset is not None:
        if data_args.val_size > 1e-6:

            val_size = (
                int(data_args.val_size)
                if data_args.val_size > 1
                else data_args.val_size
            )
            dataset = dataset.train_test_split(test_size=val_size, seed=seed)
            dataset_dict = {
                "train": dataset["train"],
                "validation": dataset["test"],
            }
        else:
            dataset_dict["train"] = dataset

    if eval_dataset is not None:
        if isinstance(eval_dataset, dict):
            dataset_dict.update(
                {f"validation_{name}": data for name, data in eval_dataset.items()}
            )
        else:
            dataset_dict["validation"] = eval_dataset

    return DatasetDict(dataset_dict)


def get_dataset_list(
    dataset_names: Optional[list[str]], dataset_dir: str
) -> list["DatasetAttr"]:
    r"""Get the attributes of the datasets."""
    if dataset_names is None:
        dataset_names = []

    if dataset_dir == "ONLINE":
        dataset_info = None
    else:
        if dataset_dir.startswith("REMOTE:"):
            config_path = hf_hub_download(
                repo_id=dataset_dir[7:], filename=DATA_CONFIG, repo_type="dataset"
            )
        else:
            config_path = os.path.join(dataset_dir, DATA_CONFIG)

        try:
            with open(config_path) as f:
                dataset_info = json.load(f)
        except Exception as err:
            if len(dataset_names) != 0:
                raise ValueError(f"Cannot open {config_path} due to {str(err)}.")

            dataset_info = None

    dataset_list: list[DatasetAttr] = []
    for name in dataset_names:
        if dataset_info is None:  # dataset_dir is ONLINE
            load_from = "ms_hub" if use_modelscope() else "hf_hub"
            dataset_attr = DatasetAttr(load_from, dataset_name=name)
            dataset_list.append(dataset_attr)
            continue

        if name not in dataset_info:
            raise ValueError(f"Undefined dataset {name} in {DATA_CONFIG}.")

        has_hf_url = "hf_hub_url" in dataset_info[name]
        has_ms_url = "ms_hub_url" in dataset_info[name]

        if has_hf_url or has_ms_url:
            if has_ms_url and (use_modelscope() or not has_hf_url):
                dataset_attr = DatasetAttr(
                    "ms_hub", dataset_name=dataset_info[name]["ms_hub_url"]
                )
            else:
                dataset_attr = DatasetAttr(
                    "hf_hub", dataset_name=dataset_info[name]["hf_hub_url"]
                )
        else:
            dataset_attr = DatasetAttr(
                "file", dataset_name=dataset_info[name]["file_name"]
            )

        dataset_attr.join(dataset_info[name])
        dataset_list.append(dataset_attr)

    return dataset_list


def _load_single_dataset(
    dataset_attr: "DatasetAttr",
    model_args: "ModelArguments",
    data_args: "DataArguments",
    training_args: "Seq2SeqTrainingArguments",
) -> Union["Dataset", "IterableDataset"]:
    r"""Load a single dataset and aligns it to the standard format."""
    logger.info_rank0(f"Loading dataset {dataset_attr}...")
    data_path, data_name, data_dir, data_files = None, None, None, None
    if dataset_attr.load_from in ["hf_hub", "ms_hub"]:
        data_path = dataset_attr.dataset_name
        data_name = dataset_attr.subset
        data_dir = dataset_attr.folder

    elif dataset_attr.load_from == "file":
        data_files = []
        local_path = os.path.join(data_args.dataset_dir, dataset_attr.dataset_name)
        if os.path.isdir(local_path):  # is directory
            for file_name in os.listdir(local_path):
                data_files.append(os.path.join(local_path, file_name))
        elif os.path.isfile(local_path):  # is file
            data_files.append(local_path)
        else:
            raise ValueError(f"File {local_path} not found.")

        data_path = FILEEXT2TYPE.get(os.path.splitext(data_files[0])[-1][1:], None)
        if data_path is None:
            raise ValueError(
                "Allowed file types: {}.".format(",".join(FILEEXT2TYPE.keys()))
            )

        if any(
            data_path != FILEEXT2TYPE.get(os.path.splitext(data_file)[-1][1:], None)
            for data_file in data_files
        ):
            raise ValueError("File types should be identical.")
    else:
        raise NotImplementedError(f"Unknown load type: {dataset_attr.load_from}.")

    if dataset_attr.load_from == "ms_hub":
        check_version("modelscope>=1.11.0", mandatory=True)
        from modelscope import MsDataset  # type: ignore
        from modelscope.utils.config_ds import MS_DATASETS_CACHE  # type: ignore

        cache_dir = model_args.cache_dir or MS_DATASETS_CACHE
        dataset = MsDataset.load(
            dataset_name=data_path,
            subset_name=data_name,
            data_dir=data_dir,
            data_files=data_files,
            split=dataset_attr.split,
            cache_dir=cache_dir,
            token=model_args.ms_hub_token,
            use_streaming=data_args.streaming,
        )
        if isinstance(dataset, MsDataset):
            dataset = dataset.to_hf_dataset()
    else:
        dataset = load_dataset(
            path=data_path,
            name=data_name,
            data_dir=data_dir,
            data_files=data_files,
            split=dataset_attr.split,
            cache_dir=model_args.cache_dir,
            token=model_args.hf_hub_token,
            num_proc=data_args.preprocessing_num_workers,
            trust_remote_code=model_args.trust_remote_code,
        )

    if dataset_attr.num_samples is not None:
        target_num = dataset_attr.num_samples
        indexes = np.random.permutation(len(dataset))[
            :target_num
        ]  # all samples should be included
        target_num -= len(indexes)
        if target_num > 0:
            expand_indexes = np.random.choice(len(dataset), target_num)
            indexes = np.concatenate((indexes, expand_indexes), axis=0)

        assert len(indexes) == dataset_attr.num_samples, "Sample num mismatched."
        dataset = dataset.select(indexes)
        logger.info_rank0(
            f"Sampled {dataset_attr.num_samples} examples from dataset {dataset_attr}."
        )

    if data_args.max_samples is not None:  # truncate dataset
        max_samples = min(data_args.max_samples, len(dataset))
        dataset = dataset.select(range(max_samples))

    return align_dataset(dataset, dataset_attr, data_args, training_args)


def merge_dataset(
    all_datasets: list[Union["Dataset", "IterableDataset"]],
    data_args: "DataArguments",
) -> Union["Dataset", "IterableDataset"]:
    r"""Merge multiple datasets to a unified dataset."""
    if len(all_datasets) == 1:
        return all_datasets[0]

    return concatenate_datasets(all_datasets)


def _get_merged_dataset(
    dataset_names: Optional[list[str]],
    model_args: "ModelArguments",
    data_args: "DataArguments",
    training_args: "Seq2SeqTrainingArguments",
    return_dict: bool = False,
) -> Optional[Union["Dataset", "IterableDataset", dict[str, "Dataset"]]]:
    r"""Return the merged datasets in the standard format."""
    if dataset_names is None:
        return None

    datasets = {}
    for dataset_name, dataset_attr in zip(
        dataset_names, get_dataset_list(dataset_names, data_args.dataset_dir)
    ):
        datasets[dataset_name] = _load_single_dataset(
            dataset_attr, model_args, data_args, training_args
        )

    if return_dict:
        return datasets
    else:
        return merge_dataset(list(datasets.values()), data_args)


def _get_unprocessed_dataset(
    dataset: Optional[Union["Dataset", "IterableDataset"]],
    training_args: "Seq2SeqTrainingArguments",
    is_eval: bool = False,
) -> Optional[Union["Dataset", "IterableDataset"]]:
    r"""Preprocesses the dataset, including format checking and tokenization."""
    if dataset is None:
        return None

    if training_args.should_log:
        try:
            print("eval example:" if is_eval else "training example:")
            print(next(iter(dataset)))
        except StopIteration:
            raise RuntimeError(
                "Cannot find valid samples, check `data/README.md` for the data format."
            )

    return dataset


def get_dataset(
    model_args: "ModelArguments",
    data_args: "DataArguments",
    training_args: "Seq2SeqTrainingArguments",
) -> "DatasetModule":
    r"""Get the train dataset and optionally gets the evaluation dataset."""
    # Load preprocessed dataset if path exists
    if data_args.save_dir is not None:
        if has_preprocessed_data(data_args.save_dir):
            logger.warning_rank0(
                "Loading dataset from disk will ignore other data arguments."
            )
            preprocessed_data = load_from_disk(data_args.save_dir)
            dataset_module = get_dataset_module(preprocessed_data)

            logger.info_rank0(f"Loaded preprocessed dataset from {data_args.save_dir}.")
            return dataset_module

    # Load and preprocess dataset
    with training_args.main_process_first(
        desc="load dataset", local=(not data_args.data_shared_file_system)
    ):
        dataset = _get_merged_dataset(
            data_args.dataset, model_args, data_args, training_args
        )
        eval_dataset = _get_merged_dataset(
            data_args.eval_dataset,
            model_args,
            data_args,
            training_args,
            return_dict=data_args.eval_on_each_dataset,
        )

    with training_args.main_process_first(
        desc="pre-process dataset", local=(not data_args.data_shared_file_system)
    ):
        dataset = _get_unprocessed_dataset(
            dataset,
            training_args,
            is_eval=False,
        )
        if isinstance(eval_dataset, dict):
            for eval_name, eval_data in eval_dataset.items():
                eval_dataset[eval_name] = _get_unprocessed_dataset(
                    eval_data,
                    training_args,
                    is_eval=True,
                )
        else:
            eval_dataset = _get_unprocessed_dataset(
                eval_dataset,
                training_args,
                is_eval=True,
            )

        dataset_dict = split_dataset(
            dataset, eval_dataset, data_args, seed=training_args.seed
        )
        if data_args.save_dir is not None:  # save preprocessed dataset to disk
            if training_args.should_save:
                dataset_dict.save_to_disk(data_args.save_dir)
                logger.info_rank0(
                    f"Preprocessed dataset is saved at {data_args.save_dir}."
                )
                logger.info_rank0(
                    f"Please launch the training with `save_dir: {data_args.save_dir}`."
                )
            sys.exit(0)

        return get_dataset_module(dataset_dict)
