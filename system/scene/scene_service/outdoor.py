# SPDX-License-Identifier: MulanPSL-2.0
"""Scene's outdoor map facade; geometry stays in its map service."""
import asyncio
from collections import OrderedDict
import time
import uuid

from starlette.responses import HTMLResponse, JSONResponse
from starlette.routing import Route


class OutdoorScene:
    def __init__(self, clients, binding, annotations, meta, registry, lock, hold, navigation=None):
        self.clients, self.binding, self.annotations, self.meta = clients, binding, annotations, meta
        self.registry, self.lock, self.hold = registry, lock, hold
        self.navigation = navigation
        self.previews = OrderedDict()

    async def idle(self):
        state = await asyncio.to_thread(self.clients.outdoor, "status")
        if not state.get("ok"):
            raise ValueError("Cannot confirm outdoor recording state: " + state.get("detail", "unavailable"))
        await self.navigation_idle()
        session = state.get("session") or {}
        if session.get("state") in ("recording", "paused"):
            raise ValueError("Finish outdoor recording before changing maps")

    async def navigation_idle(self):
        if self.navigation and self.navigation.execution_enabled:
            nav = await asyncio.to_thread(self.navigation.status)
            task = nav.get("task") or {}
            if task.get("state") in ("accepted","planning","navigating","replanning","canceling"):
                raise ValueError("Cancel navigation and confirm stop before changing maps")

    async def synchronize(self):
        if self.binding.get("map_type") != "outdoor":
            return
        async with self.lock:
            status = await asyncio.to_thread(self.clients.outdoor, "status")
            if not status.get("ok"):
                self.hold["reason"] = "Outdoor provider unavailable; map state is unconfirmed"
                return
            if status.get("map_id"):
                try:
                    await asyncio.to_thread(self.clients.check_identity, "outdoor", status["map_id"])
                except ValueError as error:
                    self.hold["reason"] = str(error)
                    return
            if (self.binding.get("map_id") == status.get("map_id")
                    and self.binding.get("version") == status.get("version")
                    and not self.hold.get("reason")):
                return
            self.hold["reason"] = "Waiting for matching outdoor route map"
            if not status.get("map_id"):
                return
            try:
                await asyncio.to_thread(self.clients.check_identity, "outdoor", status["map_id"])
                snapshot = await asyncio.to_thread(self.clients.outdoor, "get_map")
                if not snapshot.get("ok") or snapshot.get("map_id") != status["map_id"] or snapshot.get("version") != status.get("version"):
                    return
                await self.bind(snapshot, status)
            except (ValueError, OSError) as error:
                self.hold["reason"] = str(error)

    async def bind(self, snapshot, status):
        if self.navigation:
            await asyncio.to_thread(self.navigation.bind_map, snapshot)
        previous = self.meta.read_routes(snapshot["map_id"]) if self.meta else None
        if self.meta and self.meta.read(snapshot["map_id"]):
            raise ValueError("This map ID already owns indoor Scene data")
        async with self.registry.lock():
            self.registry.clear_objects()
        if self.annotations:
            # An immutable outdoor ID fixes its origin across service restarts.
            self.annotations.rebind(snapshot["map_id"], generation=None)
            if previous and (previous.get("version") != snapshot["version"]
                             or previous.get("origin_wgs84") != snapshot["origin_wgs84"]):
                self.annotations.mark_all_stale("Outdoor coordinate identity changed; confirm annotations")
        if self.meta:
            await asyncio.to_thread(self.meta.write_routes, snapshot["map_id"], snapshot)
        self.binding.update(map_id=snapshot["map_id"], map_type="outdoor", mode="localization",
                            generation=status.get("generation"), source="outdoor_lifecycle",
                            version=snapshot["version"], origin_wgs84=snapshot["origin_wgs84"])
        self.hold["reason"] = None

    async def watch(self):
        while True:
            try:
                await self.synchronize()
            except Exception as error:
                self.hold["reason"] = str(error)
            await asyncio.sleep(.5)

    async def state(self, request):
        await self.synchronize()
        status = await asyncio.to_thread(self.clients.outdoor, "status")
        snapshot = await asyncio.to_thread(self.clients.outdoor, "get_map")
        draft = None
        if status.get("draft_ready"):
            draft = await asyncio.to_thread(self.clients.outdoor, "get_map", {"draft": True})
        matches = (self.binding.get("map_type") == "outdoor"
                   and self.binding.get("map_id") == snapshot.get("map_id")
                   and self.binding.get("version") == snapshot.get("version")
                   and not self.hold.get("reason"))
        # A new recording has coordinates before it has a saved map identity.
        # Keep its position separate from the position on the active Scene map.
        collecting = (status.get("session") or {}).get("state") in ("recording", "paused")
        capture_pose = status.get("pose") if collecting else None
        if not matches:
            status["pose"] = None
        navigation = {"available":False,"ready":False}
        if self.navigation:
            try:
                navigation = await asyncio.to_thread(self.navigation.status)
                navigation["available"] = True
                pose = navigation.get("pose")
                if (matches and pose and pose.get("age_s",99.) < 1.
                        and navigation.get("coordinate_reference_matches_map")
                        and snapshot.get("origin_wgs84") == navigation.get("coordinate_reference",{}).get("origin_wgs84")):
                    status["pose"] = pose
            except Exception as error:
                navigation["detail"] = str(error)
        return JSONResponse(dict(ok=status.get("ok", False), status=status, map=snapshot,
            draft=draft, capture_pose=capture_pose, is_active=matches, binding=dict(self.binding),
            navigation=navigation,
            places=[dict(name=a["name"],x=a["points"][0][0],y=a["points"][0][1],enabled=not a.get("stale"))
                    for a in (self.annotations.list_json() if matches and self.annotations else []) if a["kind"]=="poi"],
            annotations=self.annotations.list_json() if matches and self.annotations else []))

    async def operation(self, request):
        op = request.path_params["op"]
        if op not in ("capture", "preview_merge", "apply_merge", "plan_route", "execute", "cancel"):
            return JSONResponse(dict(ok=False, detail="Unknown outdoor operation"), status_code=404)
        try:
            body = await request.json()
            if not isinstance(body, dict):
                raise ValueError("Expected an object")
            async with self.lock:
                if op == "execute":
                    result = await self.execute(body)
                elif op == "cancel":
                    if not self.navigation:
                        raise ValueError("Outdoor execution is not configured")
                    result = {"ok":True,"task":await asyncio.to_thread(self.navigation.cancel,body.get("run_id"))}
                else:
                    if op == "capture" and body.get("action") in ("start", "resume"):
                        await self.navigation_idle()
                    if op == "plan_route" and self.navigation and not body.get("start"):
                        try:
                            nav = await asyncio.to_thread(self.navigation.status)
                            if nav.get("coordinate_reference_matches_map") and nav.get("pose",{}) and 0 <= nav["pose"].get("age_s",99.) < 1.:
                                body = {**body,"start":{k:nav["pose"][k] for k in ("x","y")}}
                        except Exception:
                            pass  # Mapping can still preview from its own fresh GNSS input.
                    result = await asyncio.to_thread(self.clients.outdoor, op, body)
                    if result.get("ok") and op == "plan_route":
                        token = uuid.uuid4().hex
                        self.previews[token] = (time.monotonic(),dict(body),dict(result))
                        while len(self.previews)>16:self.previews.popitem(last=False)
                        result["preview_id"] = token
                    if result.get("ok") and op == "capture" and body.get("action") == "start" and body.get("new_map"):
                        self.hold["reason"] = "An outdoor draft is being collected; load a saved map before editing semantics"
            return JSONResponse(result, status_code=200 if result.get("ok") else 409)
        except (ValueError, KeyError) as error:
            return JSONResponse(dict(ok=False, detail=str(error)), status_code=400)

    async def execute(self, body):
        if not self.navigation:
            raise ValueError("Outdoor execution is not configured")
        token = body.get("preview_id")
        if token not in self.previews:
            raise ValueError("Preview this goal again before execution")
        created, payload, planned = self.previews[token]
        if time.monotonic()-created>120.:
            raise ValueError("Preview expired; preview this goal again")
        current = await asyncio.to_thread(self.clients.outdoor,"get_map")
        if (not current.get("ok") or current["map_id"]!=planned["map_id"]
                or current["version"]!=planned["version"]):
            raise ValueError("Loaded map changed; preview again")
        state = await asyncio.to_thread(self.clients.outdoor,"status")
        if (state.get("session") or {}).get("state") in ("recording","paused"):
            raise ValueError("Finish recording before navigation")
        nav = await asyncio.to_thread(self.navigation.status)
        if not nav.get("ready") or not nav.get("pose"):
            raise ValueError("Navigation is not ready: "+str(nav.get("not_ready_reasons",[])))
        payload = {**payload,"start":{k:nav["pose"][k] for k in ("x","y")}}
        route = await asyncio.to_thread(self.clients.outdoor,"plan_route",payload)
        if not route.get("ok"):
            raise ValueError(route.get("detail","Planning failed"))
        source = {"preview_id":token,"map_id":planned["map_id"],"version":planned["version"],
                  "goal":planned["goal"]}
        accepted = await asyncio.to_thread(self.navigation.submit_reference,route,body.get("request_id"),source)
        return {"ok":True,**accepted,"route":{**route,**accepted,"preview_id":token}}

    async def save(self, map_id, body):
        await self.idle()
        snapshot = await asyncio.to_thread(self.clients.outdoor, "get_map", {"draft": True})
        if not snapshot.get("ok"):
            return JSONResponse(snapshot, status_code=409)
        result = await asyncio.to_thread(self.clients.standard, "outdoor", "save_map",
                                         {"map_id": map_id, "note": body.get("note", "")})
        if result.get("ok"):
            snapshot["map_id"] = map_id
            if self.meta:
                await asyncio.to_thread(self.meta.write_routes, map_id, snapshot)
            parent = snapshot.get("parent_map_id")
            if parent and self.annotations:
                await asyncio.to_thread(self.annotations.copy_map, parent, map_id, generation=None)
        return JSONResponse(result, status_code=200 if result.get("ok") else 409)

    async def load(self, map_id, body):
        await self.idle()
        expected = await asyncio.to_thread(self.clients.outdoor, "get_map", {"map_id": map_id})
        if not expected.get("ok"):
            return JSONResponse(expected, status_code=409)
        self.hold["reason"] = "Outdoor map loading has not completed"
        result = await asyncio.to_thread(self.clients.standard, "outdoor", "load_map",
            {"map_id": map_id, "mode": "localization", "has_initial_pose": bool(body.get("has_initial_pose", False))})
        if not result.get("ok"):
            return JSONResponse(result, status_code=409)
        snapshot = await asyncio.to_thread(self.clients.outdoor, "get_map")
        if snapshot.get("map_id") != map_id or snapshot.get("version") != expected.get("version"):
            return JSONResponse(dict(ok=False, detail="Loaded route identity/version did not match"), status_code=502)
        status = await asyncio.to_thread(self.clients.outdoor, "status")
        await self.bind(snapshot, status)
        return JSONResponse(dict(ok=True, detail="Outdoor route map ready", map_id=map_id,
                                 version=snapshot["version"]))

    async def delete(self, map_id, body):
        await self.idle()
        if self.binding.get("map_type") == "outdoor" and self.binding.get("map_id") == map_id:
            raise ValueError("Load another map before deleting this one")
        result = await asyncio.to_thread(self.clients.standard, "outdoor", "delete_map", {"map_id": map_id})
        if result.get("ok"):
            if self.annotations:
                self.annotations.delete_map(map_id)
            if self.meta:
                self.meta.delete(map_id)
        return JSONResponse(result, status_code=200 if result.get("ok") else 409)

    def routes(self, page):
        async def outdoor_page(request):
            return HTMLResponse(page())
        return [Route("/outdoor", outdoor_page, methods=["GET"]),
                Route("/api/outdoor/state", self.state, methods=["GET"]),
                Route("/api/outdoor/{op}", self.operation, methods=["POST"])]
