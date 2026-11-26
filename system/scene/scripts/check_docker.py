#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""GPU and perception checks executed inside a candidate Scene image."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))


def gpu_info(inference: str) -> dict:
    if inference == 'torch':
        import torch
        import torchvision

        cpu = os.environ.get('SCENE_CG_FORCE_CPU') == '1'
        device = 'cpu' if cpu else 'cuda'
        assert cpu or torch.cuda.is_available(), 'CUDA is unavailable in the container'
        x = torch.ones((1, 3, 16, 16), device=device)
        assert torch.nn.functional.conv2d(x, torch.ones((4, 3, 3, 3), device=device)).isfinite().all().item()
        boxes = torch.tensor([[0., 0., 8., 8.], [1., 1., 9., 9.]], device=device)
        assert torchvision.ops.nms(boxes, torch.tensor([.9, .8], device=device), .5).tolist() == [0]
        if cpu:
            return dict(name='cpu', sm=None, multiprocessors=0, memory=0)
        torch.cuda.synchronize()
        p = torch.cuda.get_device_properties(0)
        return dict(name=p.name, sm=f'{p.major}{p.minor}', multiprocessors=p.multi_processor_count,
                    memory=p.total_memory)
    from scene_service.ingest.perception_engines import YOLO_NC_MAX
    from scene_service.ingest.perception_concept_graphs import _resolved_classes
    assert len(_resolved_classes()) <= YOLO_NC_MAX, 'Too many TensorRT classes'
    import tensorrt as trt
    from cuda.bindings import runtime as rt

    policy = json.loads((PKG / 'container-policy.json').read_text())
    assert trt.__version__.startswith(policy['profile']['docker']['tensorrt_prefix']), trt.__version__
    with trt.Builder(trt.Logger(trt.Logger.ERROR)) as builder:
        assert builder is not None, 'TensorRT cannot initialize CUDA'
    status, p = rt.cudaGetDeviceProperties(0)
    assert status == rt.cudaError_t.cudaSuccess, status
    name = p.name.decode() if isinstance(p.name, bytes) else str(p.name)
    return dict(name=name.rstrip('\0'), sm=f'{p.major}{p.minor}',
                multiprocessors=p.multiProcessorCount, memory=p.totalGlobalMem)


def perception_check(inference: str) -> None:
    import numpy as np
    import open3d as o3d
    from scene_service.ingest.perception_concept_graphs import _resolved_classes
    from scene_service.ingest.perception_models import load_perception

    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.eye(3)))
    assert len(cloud.voxel_down_sample(.01).points) == 3
    backend = os.environ.get('SCENE_PERCEPTION_BACKEND') or 'concept_graphs'
    bundle = load_perception(_resolved_classes(), profile=os.environ.get('SCENE_PROFILE') or 'lite', backend=backend)
    assert bundle is not None, 'Perception model loading failed'
    try:
        image = np.full((480, 640, 3), 127, dtype=np.uint8)
        boxes = np.array([[32, 32, 128, 128]], dtype=np.float32)
        detections = bundle.detect(image, .3)
        assert detections.ndim == 2 and detections.shape[1] == 6
        masks = bundle.segment(image, boxes)
        assert masks.shape == (1, 480, 640), masks.shape
        features = bundle.clip_feats(image, boxes)
        text = bundle.embed_text(['chair', 'table'])
        assert features.ndim == 2 and features.shape[0] == 1 and text.shape == (2, features.shape[1])
        np.testing.assert_allclose(np.linalg.norm(features, axis=1), 1, atol=1e-5)
        np.testing.assert_allclose(np.linalg.norm(text, axis=1), 1, atol=1e-5)
    finally:
        bundle.close()
    if inference == 'tensorrt':
        assert not {'torch', 'ultralytics', 'open_clip'}.intersection(sys.modules), 'Torch entered TensorRT runtime'


def storage_viewer_check() -> None:
    import rerun as rr
    from scene_service.persistence import verify_local_store

    verify_local_store()
    recording = rr.RecordingStream('scene-image-check')
    recording.log('point', rr.Points3D([[0., 0., 0.]]))
    for name in ('index.js', 're_viewer.js', 're_viewer_bg.wasm'):
        assert (Path('/opt/rerun-web-viewer') / name).is_file(), name


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inference', choices=('torch', 'tensorrt'), required=True)
    parser.add_argument('--quick', action='store_true', help='check only GPU access for startup')
    parser.add_argument('--imports-only', action='store_true', help='validate packaged service imports without a GPU')
    args = parser.parse_args()
    os.environ['SCENE_CG_INFERENCE'] = args.inference
    if args.imports_only:
        import platform
        policy = json.loads((PKG / 'container-policy.json').read_text())['profile']
        assert platform.freedesktop_os_release()['VERSION_ID'] == policy['ubuntu']
        assert f'{sys.version_info.major}.{sys.version_info.minor}' == policy['python']
        storage_viewer_check()
        import open3d  # noqa: F401
        import scene_service.service  # noqa: F401
        assert 'torch' not in sys.modules, 'Service import must remain lazy'
        print('Packaged Scene imports OK')
        return
    info = gpu_info(args.inference)
    if not args.quick:
        perception_check(args.inference)
    print(json.dumps(info), flush=True)


if __name__ == '__main__':
    main()
