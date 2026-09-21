# SPDX-License-Identifier: MulanPSL-2.0

from types import SimpleNamespace

from robonix_api.atlas_types import from_pb_contract


def _pb_contract(*, llm_callable=None):
    """Build the protobuf-shaped minimum needed by from_pb_contract."""
    values = {
        "id": "robonix/service/example/run",
        "version": "1",
        "kind": 2,
        "mode": "rpc",
        "io_msg_type": "",
        "io_srv_type": "example/srv/Run",
        "source_toml_path": "example.v1.toml",
        "description": "Run an example.",
        "cross_namespace": False,
        "msg_fields": (),
        "srv_request_fields": (),
        "srv_response_fields": (),
    }
    if llm_callable is not None:
        values["llm_callable"] = llm_callable
    contract = SimpleNamespace(**values)
    contract.HasField = lambda name: name == "llm_callable" and llm_callable is not None
    return contract


def test_contract_descriptor_preserves_explicit_llm_visibility():
    assert not from_pb_contract(_pb_contract(llm_callable=False)).llm_callable
    assert from_pb_contract(_pb_contract(llm_callable=True)).llm_callable


def test_contract_descriptor_defaults_old_wire_to_visible():
    assert from_pb_contract(_pb_contract()).llm_callable
