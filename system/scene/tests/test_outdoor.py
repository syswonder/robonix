# SPDX-License-Identifier: MulanPSL-2.0
"""Outdoor facade identity and cross-provider regressions, without ROS."""
import copy
import json

import pytest
from starlette.testclient import TestClient

from scene_service.annotations import AnnotationStore
from scene_service.map_meta import MapMetaStore
from scene_service.state.object_registry import ObjectRegistry
from scene_service.web import make_app


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
