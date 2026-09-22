# SPDX-License-Identifier: MulanPSL-2.0
"""Every scene tool that exists is a tool the agent can call.

`find` and `go_to` were implemented, carried v1 contracts, and passed their
own tests -- and were uncallable. The declaration loop walked a tuple
written by hand in two places, and adding a tool meant remembering to add a
line to each. Forgetting raises nothing: the tool is simply absent, and it
surfaces weeks later as "the agent can't do that".

So the check is the one the tuple could not make: whatever carries a
contract gets declared, and whatever has a contract file has a handler.
"""
from pathlib import Path

import pytest


def _contract_ids_from_capabilities() -> set:
    """The contracts this package ships, read off the toml filenames.

    Filenames rather than parsed contents: the id inside each file is
    `robonix/system/scene/<name>` and the name is the filename, so the two
    disagreeing is itself worth a failure.
    """
    here = Path(__file__).resolve().parents[3] / "capabilities/system/scene"
    if not here.is_dir():
        pytest.skip(f"capability directory not found at {here}")
    return {f"robonix/system/scene/{p.name.split('.')[0]}"
            for p in here.glob("*.v1.toml")}


def test_every_handler_with_a_contract_is_declared():
    from scene_service.service import declarable_scene_tools

    declared = {getattr(fn, "_robonix_contract_id") for fn in
                declarable_scene_tools()}
    assert "robonix/system/scene/find" in declared
    assert "robonix/system/scene/go_to" in declared


def test_declaration_covers_every_shipped_contract():
    """A contract file with no declared handler is a capability the package
    advertises and cannot serve."""
    from scene_service.service import declarable_scene_tools

    declared = {getattr(fn, "_robonix_contract_id") for fn in
                declarable_scene_tools()}
    shipped = _contract_ids_from_capabilities()
    missing = sorted(shipped - declared)
    assert not missing, f"contracts shipped but never declared: {missing}"


def test_no_contract_is_declared_twice():
    """A handler re-exported under a second name is one tool. Declaring it
    twice leaves atlas holding two rows for one capability."""
    from scene_service.service import declarable_scene_tools

    tools = declarable_scene_tools()
    ids = [getattr(fn, "_robonix_contract_id") for fn in tools]
    assert len(ids) == len(set(ids))


def test_the_order_does_not_depend_on_definition_order():
    """The log line counts them and atlas receives them in this order;
    neither should move when a function is moved in the file."""
    from scene_service.service import declarable_scene_tools

    ids = [getattr(fn, "_robonix_contract_id") for fn in
           declarable_scene_tools()]
    assert ids == sorted(ids)
