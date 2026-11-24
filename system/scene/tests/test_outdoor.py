# SPDX-License-Identifier: MulanPSL-2.0
"""Outdoor facade identity and cross-provider regressions, without ROS."""
import asyncio

import pytest
from starlette.testclient import TestClient

from scene_service.state.object_registry import ObjectRegistry


def test_navigation_poll_failure_does_not_hold_indoor_map(monkeypatch, caplog):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from scene_service.outdoor import OutdoorScene

    hold = {"reason": None}
    navigation = SimpleNamespace(status=Mock(side_effect=OSError("navigation unavailable")))
    facade = OutdoorScene(None, {"map_type": "indoor"}, None, None, None, None, hold, navigation)

    async def finish(_):
        raise asyncio.CancelledError()

    monkeypatch.setattr(asyncio, "sleep", finish)
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(facade.watch())
    navigation.status.assert_called_once()
    assert hold["reason"] is None
    assert "Outdoor navigation status unavailable" in caplog.text


def test_outdoor_load_does_not_wait_for_grid_and_tabs_do_not_load(scene):
    client,backend,binding,annotations,calls=scene
    for path in ('/','/outdoor','/maps?map_type=outdoor'):
        assert client.get(path).status_code==200
    assert not any(op=='load_map' for _,op,_,_ in calls)
    result=client.post('/api/maps/load',json=dict(map_id='outside',map_type='outdoor'))
    assert result.status_code==200,result.text
    assert binding['map_type']=='outdoor' and binding['version']=='v1'
    assert annotations.map_id=='outside'
    assert ('map','load_map','outdoor-map',dict(map_id='outside',mode='localization',has_initial_pose=False)) in calls
    assert client.get('/api/state').json()['route_map']['version']=='v1'
    assert 'Load an indoor map' in client.get('/').text
    assert not any(op=='occupancy_grid' for _,op,_,_ in calls)
    assert client.post('/api/maps/save',json={'map_id':'inside'}).status_code==409


def test_untyped_load_infers_map_type_from_registration(scene, monkeypatch):
    client,backend,binding,annotations,calls=scene
    result=client.post('/api/maps/load',json=dict(map_id='outside'))
    assert result.status_code==200,result.text
    assert binding['map_type']=='outdoor' and binding['version']=='v1'
    # Untyped indoor loads still route to the indoor flow (502 here only
    # because the fixture has no occupancy hub), not to a type collision.
    monkeypatch.setenv("SCENE_MAP_READY_TIMEOUT_S", "0.2")
    result=client.post('/api/maps/load',json=dict(map_id='inside'))
    assert result.status_code!=409,result.text
    # An explicit wrong type still reports the ID collision.
    assert client.post('/api/maps/load',json=dict(map_id='outside',map_type='indoor')).status_code==409


def test_cloud_http_facade_is_read_only_and_failures_are_independent(scene):
    import sys
    from pathlib import Path
    from types import SimpleNamespace
    sys.path.insert(0, str(Path(__file__).parents[4] / 'service-navigation-outdoor-rbnx'))
    from outdoor_nav.scene import SceneNavigation
    from outdoor_nav.http import ApiError
    from scene_service.outdoor import OutdoorScene
    from starlette.applications import Starlette
    client,backend,binding,annotations,calls=scene
    reader=SceneNavigation(execution_enabled=False)
    def clouds():
        raise ApiError(503,'Waiting for the lidar cloud')
    reader.clouds=clouds
    facade=OutdoorScene(None,binding,None,None,None,None,{},reader)
    cloud_client=TestClient(Starlette(routes=facade.routes(lambda:'')))
    assert cloud_client.get('/api/outdoor/clouds').status_code==503
    reader.clouds=lambda:{'ok':True,'frame_id':'base_link','count':3,'points_b64':'AAAA'}
    assert cloud_client.get('/api/outdoor/clouds').json()['count']==3
    facade_none=OutdoorScene(None,binding,None,None,None,None,{},None)
    none_client=TestClient(Starlette(routes=facade_none.routes(lambda:'')))
    assert none_client.get('/api/outdoor/clouds').status_code==503


def test_recording_and_unknown_state_block_switches(scene):
    client,backend,binding,annotations,calls=scene
    backend['recording']=True
    assert client.post('/api/maps/load',json=dict(map_id='outside',map_type='outdoor')).status_code==409
    assert client.post('/api/maps/load',json=dict(map_id='inside',map_type='indoor')).status_code==409
    backend.update(recording=False,available=False)
    assert client.post('/api/maps/load',json=dict(map_id='inside',map_type='indoor')).status_code==409
    assert not any(op=='load_map' for _,op,_,_ in calls)


def test_capture_position_visible_without_binding_to_another_map(scene):
    client,backend,binding,annotations,calls=scene
    backend.update(recording=True,pose=dict(x=2.,y=3.))
    state=client.get('/api/outdoor/state').json()
    assert not state['is_active']
    assert state['status']['pose'] is None
    assert state['capture_pose']==backend['pose']
    assert state['annotations']==[]
    assert binding['map_type']=='indoor'
    backend['recording']=False
    assert client.get('/api/outdoor/state').json()['capture_pose'] is None
    backend.update(recording=True,pose=None)
    assert client.get('/api/outdoor/state').json()['capture_pose'] is None


def test_capture_and_resume_reject_active_navigation(scene, monkeypatch):
    from types import SimpleNamespace
    from scene_service.outdoor import OutdoorScene
    previous=OutdoorScene.__init__
    def init(self,*args,**kwargs):
        previous(self,*args,**kwargs)
        self.navigation=SimpleNamespace(execution_enabled=True,status=lambda:{'task':{'state':'navigating'}})
    monkeypatch.setattr(OutdoorScene,'__init__',init)
    from scene_service.web import make_app
    client=TestClient(make_app(registry=ObjectRegistry(),map_providers={'outdoor':'outdoor-map'}))
    for action in ('start','resume'):
        response=client.post('/api/outdoor/capture',json={'action':action,'new_map':True})
        assert response.status_code==400
        assert 'confirm stop' in response.json()['detail']


def test_collision_and_external_version_change_hide_stale_semantics(scene):
    client,backend,binding,annotations,calls=scene
    backend['collision']=True
    assert client.post('/api/maps/load',json=dict(map_id='outside',map_type='outdoor')).status_code==409
    assert not any(op=='load_map' for _,op,_,_ in calls)
    backend['collision']=False
    assert client.post('/api/maps/load',json=dict(map_id='outside',map_type='outdoor')).status_code==200
    annotations.create(kind='poi',name='point',points=[[1.,0.]])
    backend['map']['version']='v2'
    state=client.get('/api/outdoor/state').json()
    assert state['binding']['version']=='v2' and state['is_active']
    assert state['annotations'][0]['stale']
    assert client.post('/api/regions',json=dict(kind='poi',name='stale',points=[[2.,0.]],map_id='outside',version='v1')).status_code==400
    backend['available']=False
    assert client.get('/api/state').json()['annotations']==[]
    assert client.delete('/api/regions/missing').status_code==409
    assert client.post('/api/annotations',json=dict(kind='poi',name='blocked',points=[[2.,0.]])).status_code==400


@pytest.mark.parametrize(("name", "goal_x"), [("快递点", 4.0), ("北门访客入口", 2.0)])
def test_pilot_named_place_preview_is_visible_but_cannot_execute(scene, monkeypatch, name, goal_x):
    from scene_service import map_clients, mcp_tools

    client, backend, _, annotations, _ = scene
    assert client.post("/api/maps/load", json={"map_id": "outside"}).status_code == 200
    monkeypatch.setattr(mcp_tools, "_OUTDOOR", client.app.state.outdoor)
    monkeypatch.setattr(client.app.state.outdoor, "preview_from_road_start", True)
    request = mcp_tools.ListOutdoorPlaces_Request(name=f" {name} ")
    assert not asyncio.run(mcp_tools.list_outdoor_places(request)).places
    saved = client.post("/api/regions", json={"kind": "poi", "name": name, "points": [[goal_x, 0.0]],
                                              "map_id": "outside", "version": "v1"})
    assert saved.status_code == 200, saved.text
    place_id = saved.json()["annotation"]["annotation_id"]
    goal = {"x": goal_x, "y": 0.0}

    original = map_clients.call

    def call(namespace, operation, payload, timeout, provider):
        if operation != "plan_route":
            return original(namespace, operation, payload, timeout, provider)
        assert payload["start"] == {"x": 0.0, "y": 0.0}
        assert payload["goal"] == goal
        return dict(ok=True, map_id="outside", version="v1", goal=payload["goal"],
                    reference_path=[payload["start"], payload["goal"]], length_m=goal_x)

    monkeypatch.setattr(map_clients, "call", call)
    result = asyncio.run(mcp_tools.list_outdoor_places(request))
    assert [found.id for found in result.places] == [place_id]
    preview = asyncio.run(mcp_tools.preview_outdoor_place(mcp_tools.PreviewOutdoorPlace_Request(
        map_id=result.map_id, version=result.version, place_id=result.places[0].id)))
    assert preview.simulated_start and preview.length_m == goal_x
    shown = client.get("/api/outdoor/state").json()["place_preview"]
    assert shown["reference_path"][-1] == goal
    assert "preview_id" not in shown
    assert client.post("/api/outdoor/execute", json={}).status_code == 400
    annotations.update(place_id, points=[[3.0, 0.0]])
    assert client.get("/api/outdoor/state").json()["place_preview"] is None
    backend["map"]["version"] = "v2"
    with pytest.raises(ValueError, match="Outdoor map changed"):
        asyncio.run(mcp_tools.preview_outdoor_place(mcp_tools.PreviewOutdoorPlace_Request(
            map_id=result.map_id, version=result.version, place_id=place_id)))


def test_pilot_outdoor_calls_use_scene_loop(monkeypatch):
    from types import SimpleNamespace
    from anyio.from_thread import start_blocking_portal
    from scene_service import mcp_tools

    monkeypatch.setattr(mcp_tools, "_OUTDOOR", None)
    monkeypatch.setattr(mcp_tools, "_OUTDOOR_LOOP", None)

    async def list_places(name):
        assert name == "快递点"
        assert asyncio.get_running_loop() is scene_loop
        return {"map_id": "outside", "version": "v1"}, []

    with start_blocking_portal() as portal:
        scene_loop = portal.call(asyncio.get_running_loop)
        portal.call(mcp_tools.attach_outdoor, SimpleNamespace(list_places=list_places))
        result = asyncio.run(mcp_tools.list_outdoor_places(mcp_tools.ListOutdoorPlaces_Request(name="快递点")))
    assert result.map_id == "outside"


def test_costmap_http_facade_is_read_only_and_failures_are_independent(scene):
    import sys
    from pathlib import Path
    from types import SimpleNamespace
    sys.path.insert(0, str(Path(__file__).parents[4] / 'service-navigation-outdoor-rbnx'))
    from outdoor_nav.scene import SceneNavigation
    from outdoor_nav.http import ApiError
    from scene_service.outdoor import OutdoorScene
    client,backend,binding,annotations,calls=scene
    reader=SceneNavigation(execution_enabled=False)
    requests=[]
    def costmap(layer):
        requests.append(layer)
        if layer=='local':raise ApiError(503,'Waiting for local costmap')
        return {'frame_id':'map','image':'global-grid','age_s':4.,'stale':True}
    reader.costmap=costmap
    facade=OutdoorScene(None,binding,None,None,None,None,{},reader)
    from starlette.applications import Starlette
    cost_client=TestClient(Starlette(routes=facade.routes(lambda:'')))
    assert cost_client.get('/api/outdoor/costmaps/global').json()['image']=='global-grid'
    assert cost_client.get('/api/outdoor/costmaps/local').status_code==503
    assert cost_client.get('/api/outdoor/costmaps/invalid').status_code==404
    assert requests==['global','local']
    assert not calls
