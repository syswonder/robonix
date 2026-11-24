# SPDX-License-Identifier: MulanPSL-2.0
"""Scene's outdoor map facade; geometry stays in its map service."""
import asyncio
from collections import OrderedDict
import json
import logging
import time
import uuid

from starlette.responses import HTMLResponse, JSONResponse
from starlette.routing import Route

from . import satellite as satellite_tiles


class OutdoorScene:
    def __init__(self, clients, binding, annotations, meta, registry, lock, hold, navigation=None,
                 preview_from_road_start=False):
        self.clients, self.binding, self.annotations, self.meta = clients, binding, annotations, meta
        self.registry, self.lock, self.hold = registry, lock, hold
        self.navigation = navigation
        self.preview_from_road_start = preview_from_road_start
        self.previews = OrderedDict()
        self.place_preview = None

    async def _place_map(self):
        snapshot = await asyncio.to_thread(self.clients.outdoor, "get_map")
        if (not snapshot.get("ok") or self.binding.get("map_type") != "outdoor"
                or self.hold.get("reason") or not self.annotations
                or snapshot["map_id"] != self.annotations.map_id
                or any(snapshot[key] != self.binding.get(key) for key in ("map_id", "version"))):
            raise ValueError("Load a saved outdoor map in Scene first")
        return snapshot

    async def list_places(self, name):
        await self.synchronize()
        async with self.lock:
            snapshot = await self._place_map()
            query = " ".join(name.split()).casefold()
            if not query:
                raise ValueError("A place name is required")
            return snapshot, [place for place in self.annotations.list() if place.kind == "poi"
                              and " ".join(place.name.split()).casefold() == query]

    async def _plan(self, body):
        if self.navigation and not body.get("start"):
            try:
                nav = await asyncio.to_thread(self.navigation.status)
                pose = nav.get("pose") or {}
                if nav.get("coordinate_reference_matches_map") and 0 <= pose.get("age_s", 99.) < 1.:
                    body = {**body, "start": {key: pose[key] for key in ("x", "y")}}
            except Exception:
                pass
        return body, await asyncio.to_thread(self.clients.outdoor, "plan_route", body)

    async def preview_place(self, map_id, version, place_id):
        await self.synchronize()
        async with self.lock:
            snapshot = await self._place_map()
            if (map_id, version) != (snapshot["map_id"], snapshot["version"]):
                raise ValueError("Outdoor map changed; look up the place again")
            place = self.annotations.get(place_id)
            if not place or place.kind != "poi" or place.stale:
                raise ValueError("The outdoor place is missing or needs reconfirmation")
            body = dict(map_id=map_id, version=version, goal=dict(zip(("x", "y"), place.points[0])))
            simulated = self.preview_from_road_start
            if simulated:
                first = next((road["points"][0] for road in snapshot["network"]["roads"]
                              if road["points"]), None)
                if first is None:
                    raise ValueError("No surveyed point is available for the simulated start")
                body["start"] = {key: first[key] for key in ("x", "y")}
            _, route = await self._plan(body)
            if not route.get("ok"):
                raise ValueError(route.get("detail", "Outdoor route planning failed"))
            self.place_preview = (time.monotonic(), place_id, place.updated_at, {
                **route, "place_name": place.name, "simulated_start": simulated,
            })
            return self.place_preview[3]

    def current_place_preview(self, snapshot):
        if self.place_preview is None:
            return None
        created, place_id, revision, route = self.place_preview
        place = self.annotations.get(place_id)
        if (time.monotonic() - created < 120 and place and not place.stale
                and place.updated_at == revision
                and all(route[key] == snapshot.get(key) for key in ("map_id", "version"))):
            return route
        return None

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
        self.place_preview = None
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
            if self.navigation:
                try:
                    await asyncio.to_thread(self.navigation.status)
                except Exception as error:
                    logging.getLogger(__name__).warning("Outdoor navigation status unavailable: %s", error)
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
            place_preview=self.current_place_preview(snapshot) if matches and not collecting else None,
            places=[dict(name=a["name"],x=a["points"][0][0],y=a["points"][0][1],enabled=not a.get("stale"))
                    for a in (self.annotations.list_json() if matches and self.annotations else []) if a["kind"]=="poi"],
            annotations=self.annotations.list_json() if matches and self.annotations else []))

    def _satellite_payload(self, snapshot):
        """Stitched satellite image for a snapshot, cached per map version.

        The cache lives next to the map sidecars in the persisted meta
        directory, so it survives restarts and browser changes; a re-recorded
        map (new version) fetches fresh tiles on the next click.
        """
        roads = (snapshot.get("network") or {}).get("roads") or []
        points = [p for r in roads for p in r.get("points", []) if "x" in p and "y" in p]
        if not points:
            raise ValueError("The outdoor map has no route points yet")
        map_id = snapshot.get("map_id") or "map"
        version = snapshot.get("version") or 0
        if self.meta is not None:
            cache_dir = self.meta.base_dir / "satellite"
            cache_dir.mkdir(parents=True, exist_ok=True)
            path = cache_dir / f"{map_id}__{version}.json"
            if path.exists():
                try:
                    cached = json.loads(path.read_text())
                except (OSError, ValueError):
                    cached = None
                if cached and cached.get("map_id") == map_id and cached.get("version") == version:
                    return {**cached, "cached": True}
        payload = satellite_tiles.stitch(snapshot.get("origin_wgs84"), points)
        if self.meta is not None:
            with open(path, "w") as file:
                json.dump({**payload, "map_id": map_id, "version": version}, file)
        return {**payload, "cached": False}

    async def satellite(self, request):
        snapshot = await asyncio.to_thread(self.clients.outdoor, "get_map")
        if not snapshot.get("ok"):
            return JSONResponse({"ok": False, "detail": snapshot.get("detail", "Outdoor map unavailable")}, status_code=503)
        try:
            result = await asyncio.to_thread(self._satellite_payload, snapshot)
            return JSONResponse({"ok": True, "cached": result["cached"], "image": result["image"],
                                 "x": result["x"], "y_top": result["y_top"],
                                 "width": result["width"], "height": result["height"]})
        except (ValueError, OSError) as error:
            return JSONResponse({"ok": False, "detail": str(error)}, status_code=502)

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
                    if op == "plan_route":
                        body, result = await self._plan(body)
                    else:
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
        async def costmap(request):
            layer = request.path_params['layer']
            if layer not in ('global', 'local'):
                return JSONResponse({'ok': False, 'detail': 'Unknown costmap layer'}, status_code=404)
            if self.navigation is None:
                return JSONResponse({'ok': False, 'detail': 'Navigation is not configured'}, status_code=503)
            try:
                result = await asyncio.to_thread(self.navigation.costmap, layer)
                return JSONResponse(result, headers={'Cache-Control': 'no-store'})
            except (ValueError, OSError) as error:
                return JSONResponse({'ok': False, 'detail': str(error)}, status_code=503)
        async def clouds(request):
            if self.navigation is None:
                return JSONResponse({'ok': False, 'detail': 'Navigation is not configured'}, status_code=503)
            try:
                result = await asyncio.to_thread(self.navigation.clouds)
                return JSONResponse(result, headers={'Cache-Control': 'no-store'})
            except (ValueError, OSError) as error:
                return JSONResponse({'ok': False, 'detail': str(error)}, status_code=503)
        return [Route("/outdoor", outdoor_page, methods=["GET"]),
                Route("/api/outdoor/state", self.state, methods=["GET"]),
                Route("/api/outdoor/satellite", self.satellite, methods=["GET"]),
                Route("/api/outdoor/costmaps/{layer}", costmap, methods=["GET"]),
                Route("/api/outdoor/clouds", clouds, methods=["GET"]),
                Route("/api/outdoor/{op}", self.operation, methods=["POST"])]
