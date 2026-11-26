# SPDX-License-Identifier: MulanPSL-2.0
"""Prepared DualMap mask filtering and point sampling; set SCENE_DUALMAP_ROOT."""
import importlib
import os

import numpy as np
import pytest


@pytest.fixture
def filter_type(monkeypatch):
    root = os.environ.get('SCENE_DUALMAP_ROOT')
    if not root:
        pytest.skip('requires prepared DualMap source and its mapping dependencies')
    from scripts.prepare_dualmap import verify
    verify(root)
    monkeypatch.syspath_prepend(root)
    return importlib.import_module('utils.object_detector').Filter


@pytest.mark.parametrize('difference', [-1, 0, 1])
@pytest.mark.parametrize('keep_larger', [False, True])
def test_one_pixel_duplicate_keeps_the_more_confident_class(filter_type, difference, keep_larger):
    import supervision as sv
    masks = np.zeros((2, 20, 20), dtype=bool)
    masks[:, 2:18, 2:18] = True
    if difference < 0:
        masks[1, 2, 2] = False
    elif difference > 0:
        masks[1, 2, 18] = True
    detector_filter = filter_type(classes=None, keep_larger=keep_larger)
    detector_filter.update_detections(sv.Detections(
        xyxy=np.array([[2, 2, 18, 18], [2, 2, 19, 18]], dtype=np.float32),
        confidence=np.array([.83, .35], dtype=np.float32),
        class_id=np.array([0, 1]), mask=masks,
    ), np.zeros((20, 20, 3), dtype=np.uint8))
    np.testing.assert_array_equal(detector_filter.filter_by_iou(), [True, False])


@pytest.mark.parametrize('keep_larger,expected', [(True, [False, True]), (False, [True, False])])
def test_distinct_mask_sizes_preserve_the_configured_preference(filter_type, keep_larger, expected):
    import supervision as sv
    masks = np.zeros((2, 20, 20), dtype=bool)
    masks[:, 2:18, 2:18] = True
    masks[1, 2:18, 18] = True
    detector_filter = filter_type(classes=None, keep_larger=keep_larger)
    detector_filter.update_detections(sv.Detections(
        xyxy=np.array([[2, 2, 18, 18], [2, 2, 19, 18]], dtype=np.float32),
        confidence=np.array([.83, .35], dtype=np.float32),
        class_id=np.array([0, 1]), mask=masks,
    ), np.zeros((20, 20, 3), dtype=np.uint8))
    np.testing.assert_array_equal(detector_filter.filter_by_iou(), expected)


def test_rasterization_tolerance_scales_with_mask_area(filter_type):
    import supervision as sv
    masks = np.ones((2, 100, 100), dtype=bool)
    masks[0, 0, :4] = False
    detector_filter = filter_type(classes=None)
    detector_filter.update_detections(sv.Detections(
        xyxy=np.array([[0, 0, 100, 100]] * 2, dtype=np.float32),
        confidence=np.array([.83, .35], dtype=np.float32),
        class_id=np.array([0, 1]), mask=masks,
    ), np.zeros((100, 100, 3), dtype=np.uint8))
    np.testing.assert_array_equal(detector_filter.filter_by_iou(), [True, False])


@pytest.mark.parametrize('classes,expected', [([1, 2], [True, False]), ([2, 1], [False, True])])
def test_equal_confidence_uses_class_id_independently_of_input_order(filter_type, classes, expected):
    import supervision as sv
    detector_filter = filter_type(classes=None)
    detector_filter.update_detections(sv.Detections(
        xyxy=np.array([[0, 0, 16, 16]] * 2, dtype=np.float32),
        confidence=np.array([.8, .8], dtype=np.float32), class_id=np.array(classes),
        mask=np.ones((2, 16, 16), dtype=bool),
    ), np.zeros((16, 16, 3), dtype=np.uint8))
    np.testing.assert_array_equal(detector_filter.filter_by_iou(), expected)


def test_one_pixel_boundary_change_preserves_the_point_sample(filter_type):
    from types import SimpleNamespace
    detector_type = importlib.import_module(filter_type.__module__).Detector
    detector = detector_type.__new__(detector_type)
    detector.cfg = SimpleNamespace(pcd_sample_ratio=.04, min_points_threshold=1,
                                   dbscan_eps=200., dbscan_min_points=3)
    detector.curr_data = SimpleNamespace(idx=7, depth=np.ones((100, 100, 1), dtype=np.float32),
                                        color=np.zeros((100, 100, 3), dtype=np.uint8),
                                        intrinsics=np.eye(3, dtype=np.float32))
    masks = np.ones((2, 100, 100), dtype=bool)
    masks[1, 0, 0] = False
    detector.process_masks(masks)
    first, second = [set(map(tuple, points)) for points in detector.masked_points]
    assert len(first) == 400 and len(second) == 399
    assert len(first & second) >= 398
    detector.curr_data.idx = 8
    detector.process_masks(masks[:1])
    assert set(map(tuple, detector.masked_points[0])) != first


def test_rgb_frame_is_converted_at_the_detection_interface(filter_type):
    from types import SimpleNamespace
    from unittest.mock import Mock
    detector_type = importlib.import_module(filter_type.__module__).Detector
    detector = detector_type.__new__(detector_type)
    detector.inference = SimpleNamespace(detect=Mock(return_value=np.array([[0, 0, 1, 1, .9, 0]])))
    image = np.full((2, 2, 3), [255, 0, 0], dtype=np.uint8)
    _, classes, labels, _ = detector.process_yolo_results(
        image, SimpleNamespace(get_classes_arr=lambda: ['red object']))
    np.testing.assert_array_equal(detector.inference.detect.call_args.args[0], image[..., ::-1])
    np.testing.assert_array_equal(image[0, 0], [255, 0, 0])
    assert classes.tolist() == [0] and labels == ['red object 0']


def test_fastsam_overlap_filter_matches_torch_without_importing_it(filter_type, monkeypatch):
    import builtins
    import torch
    import supervision as sv
    module = importlib.import_module(filter_type.__module__)
    rng = np.random.default_rng(9)
    first = rng.random((7, 24, 31)) > .4
    current = rng.random((3, 24, 31)) > .4
    first[0] = current[0]
    first[1] = False
    first[-1, :, :15] = False
    a, b = torch.from_numpy(first).flatten(1).float(), torch.from_numpy(current).flatten(1).float()
    intersection = a @ b.T
    aa, bb = a.sum(1)[:, None], b.sum(1)[None]
    keep = ~((intersection / (aa + bb - intersection).clamp_min(1e-7) > .5)
             | (intersection / aa.clamp_min(1e-7) > .6)
             | (intersection / bb.clamp_min(1e-7) > .6)).any(1).numpy()
    make = lambda masks: sv.Detections(xyxy=np.zeros((len(masks), 4), np.float32),
                                      confidence=np.arange(len(masks), dtype=np.float32),
                                      class_id=np.zeros(len(masks), int), mask=masks)
    source = make(first)
    original = builtins.__import__
    def no_torch(name, *args, **kwargs):
        if name.split('.')[0] == 'torch':
            raise AssertionError('overlap filtering must not import Torch')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', no_torch)
    detector = module.Detector.__new__(module.Detector)
    result = detector.filter_fs_detections_by_curr(source, make(current))
    np.testing.assert_array_equal(result.mask, first[keep])
    np.testing.assert_array_equal(result.confidence, source.confidence[keep])


@pytest.mark.parametrize('average_unknown', [True, False])
def test_fastsam_only_frame_keeps_unknown_observations(filter_type, average_unknown):
    from types import SimpleNamespace
    from unittest.mock import Mock
    module = importlib.import_module(filter_type.__module__)
    detector = module.Detector.__new__(module.Detector)
    detector.cfg = SimpleNamespace(use_fastsam=True, fastsam_confidence=.5, device='cpu', visualize_detection=False,
                                   use_avg_feat_for_unknown=average_unknown)
    detector.obj_classes = SimpleNamespace(get_classes_arr=lambda: ['chair', 'unknown'])
    detector.unknown_class_id = 1
    detector.class_feats_mean = np.ones(4, dtype=np.float32) / 2
    detector.curr_data = SimpleNamespace(color=np.zeros((4, 4, 3), np.uint8))
    detector.clip_model = detector.clip_tokenizer = detector.clip_preprocess = None
    detector.timing_results = {}
    detector.process_masks = Mock()
    detector.inference = SimpleNamespace(
        detect=Mock(return_value=np.empty((0, 6), np.float32)),
        segment_instances=Mock(return_value=(np.array([[0, 0, 4, 4, .9, 0]], np.float32), np.ones((1, 4, 4), bool))),
        clip_feats=Mock(return_value=np.ones((1, 4), np.float32)),
        embed_text=Mock(return_value=np.ones((1, 4), np.float32)),
    )
    detector.process_detections()
    assert detector.curr_results['class_id'].tolist() == [1]
    assert detector.curr_results['masks'].shape == (1, 4, 4)
    detector.inference.segment_instances.side_effect = RuntimeError('instance inference failed')
    with pytest.raises(RuntimeError, match='instance inference failed'):
        detector.process_detections()


def test_standalone_detector_reuses_shared_torch_and_keeps_query_aliases(filter_type, tmp_path, monkeypatch):
    from pathlib import Path
    from hydra import compose, initialize_config_dir
    from hydra.core.global_hydra import GlobalHydra
    from types import SimpleNamespace
    from unittest.mock import Mock
    from scene_service.ingest import perception_torch
    module = importlib.import_module(filter_type.__module__)
    root = os.environ['SCENE_DUALMAP_ROOT']
    monkeypatch.chdir(root)
    GlobalHydra.instance().clear()
    with initialize_config_dir(version_base=None, config_dir=str(Path(root) / 'config')):
        cfg = compose(config_name='runner_dataset', overrides=[f'output_path={tmp_path}', 'use_rerun=false', 'use_fastsam=true'])
    fake = SimpleNamespace(clip=object(), tokenizer=object(), preprocess=object(),
                           embed_text=lambda texts: np.ones((len(texts), cfg.clip.clip_length), np.float32))
    create = Mock(return_value=fake)
    monkeypatch.setattr(perception_torch, 'TorchPerception', create)
    module.ReRunVisualizer(cfg)
    detector = module.Detector(cfg)
    create.assert_called_once()
    assert detector.inference is fake
    assert detector.clip_model is fake.clip and detector.clip_tokenizer is fake.tokenizer
    assert create.call_args.kwargs['fastsam_path'] == cfg.fastsam.model_path
    injected = module.Detector(cfg, inference=fake)
    assert injected.inference is fake
    assert injected.clip_model is injected.clip_tokenizer is injected.clip_preprocess is None
    create.assert_called_once()
