import os
from typing import TYPE_CHECKING, Any, Optional, TypedDict

import torch
import transformers.dynamic_module_utils
from transformers.dynamic_module_utils import get_relative_imports
from transformers import (
    AutoConfig,
    AutoModelForCausalLM,
    AutoModelForSeq2SeqLM,
    AutoModelForTextToWaveform,
    AutoModelForVision2Seq,
    AutoProcessor,
    AutoTokenizer,
)

from . import logging
from .utils import check_version, count_parameters, is_env_enabled
from .patcher import patch_config, patch_model, patch_tokenizer, patch_processor
from .adapter import init_adapter

if TYPE_CHECKING:
    from transformers import (
        PretrainedConfig,
        PreTrainedModel,
        PreTrainedTokenizer,
        ProcessorMixin,
    )

    from ..hparams import DataArguments, ModelArguments, FinetuningArguments

logger = logging.get_logger(__name__)


class TokenizerModule(TypedDict):
    tokenizer: "PreTrainedTokenizer"
    processor: Optional["ProcessorMixin"]


def skip_check_imports() -> None:
    r"""Avoid flash attention import error in custom model files."""
    if not is_env_enabled("FORCE_CHECK_IMPORTS"):
        transformers.dynamic_module_utils.check_imports = get_relative_imports


def use_modelscope() -> bool:
    return is_env_enabled("USE_MODELSCOPE_HUB")


def try_download_model_from_other_hub(model_args: "ModelArguments") -> str:
    if not use_modelscope() or os.path.exists(model_args.model_name_or_path):
        return model_args.model_name_or_path

    if use_modelscope():
        check_version("modelscope>=1.11.0", mandatory=True)
        from modelscope import snapshot_download  # type: ignore

        revision = (
            "master"
            if model_args.model_revision == "main"
            else model_args.model_revision
        )
        return snapshot_download(
            model_args.model_name_or_path,
            revision=revision,
            cache_dir=model_args.cache_dir,
        )


def _get_init_kwargs(model_args: "ModelArguments") -> dict[str, Any]:
    r"""Get arguments to load config/tokenizer/model.

    Note: including inplace operation of model_args.
    """
    model_args.model_name_or_path = try_download_model_from_other_hub(model_args)
    return {
        "trust_remote_code": model_args.trust_remote_code,
        "cache_dir": model_args.cache_dir,
        "revision": model_args.model_revision,
        "token": model_args.hf_hub_token,
    }


def load_config(model_args: "ModelArguments") -> "PretrainedConfig":
    r"""Load model config."""
    init_kwargs = _get_init_kwargs(model_args)
    return AutoConfig.from_pretrained(model_args.model_name_or_path, **init_kwargs)


def load_tokenizer(model_args: "ModelArguments") -> "TokenizerModule":
    r"""Load pretrained tokenizer and optionally loads processor.

    Note: including inplace operation of model_args.
    """
    init_kwargs = _get_init_kwargs(model_args)
    try:
        tokenizer = AutoTokenizer.from_pretrained(
            model_args.model_name_or_path,
            use_fast=model_args.use_fast_tokenizer,
            split_special_tokens=model_args.split_special_tokens,
            padding_side="right",
            **init_kwargs,
        )
    except ValueError:  # try the fast one
        tokenizer = AutoTokenizer.from_pretrained(
            model_args.model_name_or_path,
            use_fast=True,
            padding_side="right",
            **init_kwargs,
        )
    except Exception as e:
        raise OSError("Failed to load tokenizer.") from e

    patch_tokenizer(tokenizer, model_args)

    return {"tokenizer": tokenizer}


def register_autoclass(
    config: "PretrainedConfig",
    model: "PreTrainedModel",
    tokenizer: "PreTrainedTokenizer",
):
    if "AutoConfig" in getattr(config, "auto_map", {}):
        config.__class__.register_for_auto_class()
    if "AutoModelForCausalLM" in getattr(config, "auto_map", {}):
        model.__class__.register_for_auto_class()
    if "AutoTokenizer" in tokenizer.init_kwargs.get("auto_map", {}):
        tokenizer.__class__.register_for_auto_class()


def load_model(
    tokenizer: "PreTrainedTokenizer",
    data_args: "DataArguments",
    model_args: "ModelArguments",
    finetuning_args: "FinetuningArguments",
    is_trainable: bool = False,
) -> "PreTrainedModel":
    r"""Load pretrained model."""
    init_kwargs = _get_init_kwargs(model_args)
    config = load_config(model_args)
    config.point_config["num_bins"] = data_args.num_bins
    patch_config(config, model_args, init_kwargs, is_trainable)

    init_kwargs["config"] = config
    init_kwargs["pretrained_model_name_or_path"] = model_args.model_name_or_path

    if model_args.train_from_scratch:
        model = AutoModelForCausalLM.from_config(
            config, trust_remote_code=model_args.trust_remote_code
        )
    else:
        model = AutoModelForCausalLM.from_pretrained(**init_kwargs)

    patch_model(model, model_args, is_trainable)
    register_autoclass(config, model, tokenizer)
    model = init_adapter(model, finetuning_args, is_trainable)

    if not is_trainable:
        model.requires_grad_(False)
        for param in model.parameters():
            if (
                param.data.dtype == torch.float32
                and model_args.compute_dtype != torch.float32
            ):
                param.data = param.data.to(model_args.compute_dtype)

        model.eval()
    else:
        model.train()

    trainable_params, all_param = count_parameters(model)
    if is_trainable:
        param_stats = (
            f"trainable params: {trainable_params:,} || "
            f"all params: {all_param:,} || trainable%: {100 * trainable_params / all_param:.4f}"
        )
    else:
        param_stats = f"all params: {all_param:,}"

    logger.info_rank0(param_stats)

    if model_args.print_param_status and int(os.getenv("LOCAL_RANK", "0")) == 0:
        for name, param in model.named_parameters():
            print(
                f"name: {name}, dtype: {param.dtype}, device: {param.device}, trainable: {param.requires_grad}"
            )

    return model
