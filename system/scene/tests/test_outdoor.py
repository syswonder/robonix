# SPDX-License-Identifier: MulanPSL-2.0
"""Outdoor facade identity and cross-provider regressions, without ROS."""
import copy
import json
import asyncio

import pytest
from starlette.testclient import TestClient

from scene_service.annotations import AnnotationStore
from scene_service.map_meta import MapMetaStore
from scene_service.state.object_registry import ObjectRegistry
from scene_service.web import make_app


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


@pytest.fixture
def scene(tmp_path, monkeypatch):
    from scene_service import map_clients
    snapshot=dict(map_id='outside',version='v1',map_type='outdoor',origin_wgs84=dict(latitude=30.,longitude=120.),
                  network=dict(nodes=[],roads=[dict(id='r',points=[dict(x=0.,y=0.),dict(x=5.,y=0.)],gait='flat')]))
    backend=dict(map=copy.deepcopy(snapshot),recording=False,available=True,collision=False)
    calls=[]
    def call(namespace,op,payload=None,timeout_s=20,provider_id=''):
        calls.append((namespace,op,provider_id,payload))
        if op=='list_maps':
            maps=[dict(map_id='outside' if backend['collision'] else 'inside')] if provider_id=='indoor-map' else [dict(map_id='outside')]
            return dict(ok=True,maps_json=json.dumps(maps))
        if op=='status':
            if not backend['available']:
                return dict(ok=False,detail='disconnected')
            return dict(ok=True,map_id=backend['map']['map_id'],version=backend['map']['version'],generation=1,
                        pose=backend.get('pose'),
                        session=dict(state='recording' if backend['recording'] else 'finished'))
        if op=='get_map':
            return dict(ok=True,**backend['map'])
        return dict(ok=True,detail='')
    monkeypatch.setattr(map_clients,'call',call)
    annotations=AnnotationStore(str(tmp_path/'annotations'),map_id='.live')
    binding=dict(map_id='inside',map_type='indoor',mode='localization')
    app=make_app(registry=ObjectRegistry(),anno_store=annotations,map_meta=MapMetaStore(str(tmp_path/'meta')),
                 map_binding=binding,map_providers={'indoor':'indoor-map','outdoor':'outdoor-map'})
    return TestClient(app),backend,binding,annotations,calls


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
