# SPDX-License-Identifier: MulanPSL-2.0
"""Map operations pinned to the provider owning each map type."""
import importlib
import json

from robonix_api import ATLAS


def call(namespace, op, payload=None, timeout_s=20., provider_id=""):
    import grpc
    messages = importlib.import_module("map_pb2" if namespace == "map" else "outdoor_map_pb2")
    contracts = importlib.import_module("robonix_contracts_pb2_grpc")
    method = "".join(part.title() for part in op.split("_"))
    contract = f"robonix/service/{namespace}/{op}"
    caps = ATLAS.find_capability(contract_id=contract, transport="grpc", provider_id=provider_id)
    if len(caps) != 1:
        return dict(ok=False, detail=f"Expected one provider for {contract}; configure map_providers")
    channel_ref = None
    try:
        channel_ref = ATLAS.connect_capability(consumer_id="scene", provider_id=caps[0].provider_id,
                                              contract_id=contract, transport="grpc")
        with grpc.insecure_channel(channel_ref.endpoint, options=[("grpc.enable_http_proxy", 0)]) as channel:
            base = "RobonixService"+("Map" if namespace == "map" else "MapOutdoor")+method
            stub = getattr(contracts, base+"Stub")(channel)
            body = {"request_json": json.dumps(payload or {})} if namespace == "map_outdoor" else payload or {}
            response = getattr(stub, method)(getattr(messages, method+"_Request")(**body), timeout=timeout_s)
        result = {field.name: getattr(response, field.name) for field in response.DESCRIPTOR.fields}
        if namespace == "map_outdoor" and result["ok"]:
            return {**json.loads(result["result_json"]), "ok": True}
        return result
    except (grpc.RpcError, ValueError) as error:
        return dict(ok=False, detail=str(error))
    finally:
        if channel_ref:
            channel_ref.close()


class MapClients:
    def __init__(self, providers=None):
        self.providers = dict(providers or {})

    def standard(self, kind, op, payload=None, timeout_s=20.):
        if kind not in ("indoor", "outdoor"):
            raise ValueError("Map type must be indoor or outdoor")
        if kind not in self.providers:
            return dict(ok=False, detail=f"Configure map_providers.{kind}")
        return call("map", op, payload, timeout_s, self.providers[kind])

    def outdoor(self, op, payload=None, timeout_s=20.):
        if "outdoor" not in self.providers:
            return dict(ok=False, detail="Configure map_providers.outdoor")
        return call("map_outdoor", op, payload, timeout_s, self.providers["outdoor"])

    def list(self):
        maps, failures = [], []
        for kind in self.providers:
            result = self.standard(kind, "list_maps", {})
            if result.get("ok"):
                for item in json.loads(result.get("maps_json") or "[]"):
                    maps.append({**item, "map_type": kind, "provider_id": self.providers[kind]})
            else:
                failures.append(result.get("detail", "Map provider unavailable"))
        return dict(ok=not failures, maps=maps, detail="; ".join(failures))

    def check_identity(self, kind, map_id):
        from .map_binding import sanitize_map_id
        if sanitize_map_id(map_id) != map_id:
            raise ValueError("Use a canonical map id without trailing dots or unsafe characters")
        listing = self.list()
        if not listing["ok"]:
            raise ValueError("Cannot verify map ownership: " + listing["detail"])
        owners = {m["map_type"] for m in listing["maps"]
                  if sanitize_map_id(m["map_id"]) == map_id}
        if owners - {kind}:
            raise ValueError("Indoor and outdoor maps must have distinct IDs; rename the map in its service")
