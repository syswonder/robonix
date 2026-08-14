# SPDX-License-Identifier: MulanPSL-2.0
from unittest.mock import patch

import pytest

from robonix_api.capability import Primitive, _resolve_provider_id


def test_package_id_is_the_standalone_default():
    with patch.dict("os.environ", {}, clear=True):
        assert _resolve_provider_id("orbbec_camera") == "orbbec_camera"


def test_manifest_instance_name_overrides_package_default():
    with patch.dict(
        "os.environ",
        {"RBNX_INSTANCE_NAME": "orbbec_wrist_camera"},
        clear=True,
    ):
        assert _resolve_provider_id("orbbec_camera") == "orbbec_wrist_camera"


def test_provider_uses_manifest_instance_for_runtime_identity(tmp_path):
    with patch.dict(
        "os.environ",
        {"RBNX_INSTANCE_NAME": "orbbec_wrist_camera"},
        clear=True,
    ):
        provider = Primitive(
            "orbbec_camera",
            "robonix/primitive/camera",
            pkg_root=tmp_path,
        )
    assert provider.default_id == "orbbec_camera"
    assert provider.id == "orbbec_wrist_camera"


def test_blank_instance_name_does_not_erase_package_default():
    with patch.dict("os.environ", {"RBNX_INSTANCE_NAME": "  "}, clear=True):
        assert _resolve_provider_id("orbbec_camera") == "orbbec_camera"


@pytest.mark.parametrize("instance_name", [None, "", "  "])
def test_deploy_managed_provider_requires_instance_name(instance_name):
    env = {"RBNX_DEPLOY_MANAGED": "1"}
    if instance_name is not None:
        env["RBNX_INSTANCE_NAME"] = instance_name
    with (
        patch.dict("os.environ", env, clear=True),
        pytest.raises(RuntimeError, match="RBNX_INSTANCE_NAME must be non-empty"),
    ):
        _resolve_provider_id("orbbec_camera")
