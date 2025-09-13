# SpatialLM

<!-- markdownlint-disable first-line-h1 -->
<!-- markdownlint-disable html -->
<!-- markdownlint-disable no-duplicate-header -->

<div align="center">
  <img src="figures/logo_light.png#gh-light-mode-only" width="60%" alt="SpatialLM" />
  <img src="figures/logo_dark.png#gh-dark-mode-only" width="60%" alt="SpatialLM" />
</div>
<hr style="margin-top: 0; margin-bottom: 8px;">
<div align="center" style="margin-top: 0; padding-top: 0; line-height: 1;">
    <a href="https://manycore-research.github.io/SpatialLM" target="_blank" style="margin: 2px;"><img alt="Project"
    src="https://img.shields.io/badge/🌐%20Website-SpatialLM-ffc107?color=42a5f5&logoColor=white" style="display: inline-block; vertical-align: middle;"/></a>
    <a href="https://arxiv.org/abs/2506.07491" target="_blank" style="margin: 2px;"><img alt="arXiv"
    src="https://img.shields.io/badge/arXiv-Techreport-b31b1b?logo=arxiv&logoColor=white" style="display: inline-block; vertical-align: middle;"/></a>
    <a href="https://github.com/manycore-research/SpatialLM" target="_blank" style="margin: 2px;"><img alt="GitHub"
    src="https://img.shields.io/badge/GitHub-SpatialLM-24292e?logo=github&logoColor=white" style="display: inline-block; vertical-align: middle;"/></a>
</div>
<div align="center" style="line-height: 1;">
    <a href="https://huggingface.co/manycore-research/SpatialLM1.1-Qwen-0.5B" target="_blank" style="margin: 2px;"><img alt="Hugging Face"
    src="https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-SpatialLM-ffc107?color=ffc107&logoColor=white" style="display: inline-block; vertical-align: middle;"/></a>
    <a href="https://huggingface.co/datasets/manycore-research/SpatialLM-Testset" target="_blank" style="margin: 2px;"><img alt="Dataset"
    src="https://img.shields.io/badge/%F0%9F%A4%97%20Dataset-Testset-ffc107?color=ffc107&logoColor=white" style="display: inline-block; vertical-align: middle;"/></a>
</div>

## ✨ News

- [Jun, 2025] Added finetuning instructions in [FINETUNE.md](./FINETUNE.md).
- [Jun, 2025] Check out our new models: [SpatialLM1.1-Llama-1B](https://huggingface.co/manycore-research/SpatialLM1.1-Llama-1B) and [SpatialLM1.1-Qwen-0.5B](https://huggingface.co/manycore-research/SpatialLM1.1-Qwen-0.5B), now available on Hugging Face. SpatialLM1.1 doubles the point cloud resolution, incorporates a more powerful point cloud encoder [Sonata](https://xywu.me/sonata/) and supports detection with user-specified categories.
- [Jun, 2025] SpatialLM [Technical Report](https://arxiv.org/abs/2506.07491) is now on arXiv.
- [Mar, 2025] We're excited to release the [SpatialLM-Llama-1B](https://huggingface.co/manycore-research/SpatialLM-Llama-1B) and [SpatialLM-Qwen-0.5B](https://huggingface.co/manycore-research/SpatialLM-Qwen-0.5B) on Hugging Face.
- [Mar, 2025] Initial release of SpatialLM!

## Introduction

SpatialLM is a 3D large language model designed to process 3D point cloud data and generate structured 3D scene understanding outputs. These outputs include architectural elements like walls, doors, windows, and oriented object bounding boxes with their semantic categories. Unlike previous methods that require specialized equipment for data collection, SpatialLM can handle point clouds from diverse sources such as monocular video sequences, RGBD images, and LiDAR sensors. This multimodal architecture effectively bridges the gap between unstructured 3D geometric data and structured 3D representations, offering high-level semantic understanding. It enhances spatial reasoning capabilities for applications in embodied robotics, autonomous navigation, and other complex 3D scene analysis tasks.

<div align="center">
  <video src="https://github.com/user-attachments/assets/c0218d6a-f676-41f8-ae76-bba228866306" poster="figures/cover.png"> </video>
  <p><i>SpatialLM reconstructs 3D layout from a monocular RGB video with MASt3R-SLAM. Results aligned to video with GT cameras for visualization.</i></p>
</div>

## SpatialLM Models

<div align="center">

|       **Model**        | **Download**                                                                      |
| :--------------------: | --------------------------------------------------------------------------------- |
| SpatialLM1.1-Llama-1B  | [🤗 HuggingFace](https://huggingface.co/manycore-research/SpatialLM1.1-Llama-1B)  |
| SpatialLM1.1-Qwen-0.5B | [🤗 HuggingFace](https://huggingface.co/manycore-research/SpatialLM1.1-Qwen-0.5B) |
| SpatialLM1.0-Llama-1B  | [🤗 HuggingFace](https://huggingface.co/manycore-research/SpatialLM-Llama-1B)     |
| SpatialLM1.0-Qwen-0.5B | [🤗 HuggingFace](https://huggingface.co/manycore-research/SpatialLM-Qwen-0.5B)    |

</div>

## Usage

### Installation

Tested with the following environment:

- Python 3.11
- Pytorch 2.4.1
- CUDA Version 12.4

```bash
# clone the repository
git clone https://github.com/manycore-research/SpatialLM.git
cd SpatialLM

# create a conda environment with cuda 12.4
conda create -n spatiallm python=3.11
conda activate spatiallm
conda install -y -c nvidia/label/cuda-12.4.0 cuda-toolkit conda-forge::sparsehash

# Install dependencies with poetry
pip install poetry && poetry config virtualenvs.create false --local
poetry install
# SpatialLM1.0 dependency
poe install-torchsparse # Building wheel for torchsparse will take a while
# SpatialLM1.1 dependency
poe install-sonata # Building wheel for flash-attn will take a while
```

### Inference

In the current version of SpatialLM, input point clouds are considered axis-aligned where the z-axis is the up axis. This orientation is crucial for maintaining consistency in spatial understanding and scene interpretation across different datasets and applications.
Example preprocessed point clouds, reconstructed from RGB videos using [MASt3R-SLAM](https://github.com/rmurai0610/MASt3R-SLAM), are available in [SpatialLM-Testset](#spatiallm-testset).

Download an example point cloud:

```bash
huggingface-cli download manycore-research/SpatialLM-Testset pcd/scene0000_00.ply --repo-type dataset --local-dir .
```

Run inference:

```bash
python inference.py --point_cloud pcd/scene0000_00.ply --output scene0000_00.txt --model_path manycore-research/SpatialLM1.1-Qwen-0.5B
```

### Detection with user-specified categories

SpatialLM1.1 supports object detection conditioned on user-specified categories by leveraging the flexibility of LLMs.

SpatialLM1.1 offers three variants of structured indoor modeling tasks:

- **Structured Reconstruction**: Detect walls, doors, windows, boxes.
- **Layout Estimation**: Detect walls, doors, windows.
- **3D Object Detection**: Detect boxes.

For tasks that include object box estimation, you can specify a subset of the 59 furniture categories, and the model will only predict objects within those specified categories. For example:

```bash
python inference.py --point_cloud pcd/scene0000_00.ply --output scene0000_00.txt --model_path manycore-research/SpatialLM1.1-Qwen-0.5B --detect_type object --category bed nightstand
```

### Visualization

Use `rerun` to visualize the point cloud and the predicted structured 3D layout output:

```bash
# Convert the predicted layout to Rerun format
python visualize.py --point_cloud pcd/scene0000_00.ply --layout scene0000_00.txt --save scene0000_00.rrd

# Visualize the point cloud and the predicted layout
rerun scene0000_00.rrd
```

### Evaluation

To evaluate the performance of SpatialLM, we provide `eval.py` script that reports the benchmark results on the SpatialLM-Testset in the table below in section [Benchmark Results](#benchmark-results).

Download the testset:

```bash
huggingface-cli download manycore-research/SpatialLM-Testset --repo-type dataset --local-dir SpatialLM-Testset
```

Run evaluation:

```bash
# Run inference on the PLY point clouds in folder SpatialLM-Testset/pcd with SpatialLM1.1-Qwen-0.5B model
python inference.py --point_cloud SpatialLM-Testset/pcd --output SpatialLM-Testset/pred --model_path manycore-research/SpatialLM1.1-Qwen-0.5B

# Evaluate the predicted layouts
python eval.py --metadata SpatialLM-Testset/test.csv --gt_dir SpatialLM-Testset/layout --pred_dir SpatialLM-Testset/pred --label_mapping SpatialLM-Testset/benchmark_categories.tsv
```

### Example using a custom video

We provide an example of how to use our model to estimate scene layout starting from a RGB video with the newly released [SLAM3R](https://github.com/PKU-VCL-3DV/SLAM3R) in [EXAMPLE.md](EXAMPLE.md). These steps work for MASt3R-SLAM, and other reconstruction methods as well.

### Finetune on Custom Data

For instructions on fine-tuning SpatialLM on your own data, please refer to [FINETUNE.md](./FINETUNE.md). We provide an example using the [ARKitScenes](https://github.com/apple/ARKitScenes) dataset.

## SpatialLM Testset

We provide a test set of 107 preprocessed point clouds, reconstructed from RGB videos using [MASt3R-SLAM](https://github.com/rmurai0610/MASt3R-SLAM). SpatialLM-Testset is quite challenging compared to prior clean RGBD scans datasets due to the noises and occlusions in the point clouds reconstructed from monocular RGB videos.

<div align="center">

|    **Dataset**    | **Download**                                                                       |
| :---------------: | ---------------------------------------------------------------------------------- |
| SpatialLM-Testset | [🤗 Datasets](https://huggingface.co/datasets/manycore-research/SpatialLM-TestSet) |

</div>

## Benchmark Results

### Layout Estimation

Layout estimation focuses on predicting architectural elements, i.e., walls, doors, and windows, within an indoor scene. We evaluated this task on the [Structured3D](https://structured3d-dataset.org) dataset. For [RoomFormer](https://github.com/ywyue/RoomFormer), we directly downloaded the model checkpoint. SceneScript and SpatialLM were first trained on our dataset, and further fine-tuned on Structured3D.

<div align="center">

|   **Method**    | **RoomFormer** | **SceneScript (finetuned)** | **SpatialLM1.1-Qwen-0.5B (finetuned)** |
| :-------------: | :------------: | :-------------------------: | :------------------------------------: |
| **F1 @.25 IoU** |      70.4      |            83.1             |                  86.5                  |
| **F1 @.5 IoU**  |      67.2      |            80.8             |                  84.6                  |

</div>

### 3D Object Detection

We evaluate 3D object detection on [ScanNet](http://www.scan-net.org) with annotations of 18 object categories. For [V-DETR](https://github.com/V-DETR/V-DETR), we directly download the model checkpoint. SceneScript and SpatialLM were first trained on our dataset, and further fine-tuned on ScanNet.

<div align="center">

|   **Method**    | **V-DETR** | **SceneScript (finetuned)** | **SpatialLM1.1-Qwen-0.5B (finetuned)** |
| :-------------: | :--------: | :-------------------------: | :------------------------------------: |
| **F1 @.25 IoU** |    65.1    |            49.1             |                  65.6                  |
| **F1 @.5 IoU**  |    56.8    |            36.8             |                  52.6                  |

</div>

### Zero-shot Detection on Videos

Zero-shot detection results on the challenging SpatialLM-Testset are reported in the following table:

<div align="center">

|   **Method**    | **SpatialLM1.1-Llama-1B** | **SpatialLM1.1-Qwen-0.5B** |
| :-------------: | :-----------------------: | :------------------------: |
|   **Layout**    |   **F1 @.25 IoU (2D)**    |    **F1 @.25 IoU (2D)**    |
|      wall       |           68.9            |            68.2            |
|      door       |           46.3            |            43.1            |
|     window      |           43.8            |            47.4            |
|                 |                           |                            |
|   **Objects**   |   **F1 @.25 IoU (3D)**    |    **F1 @.25 IoU (2D)**    |
|     curtain     |           34.9            |            37.0            |
|   nightstand    |           62.8            |            67.0            |
|   chandelier    |           53.5            |            36.8            |
|    wardrobe     |           29.4            |            39.6            |
|       bed       |           96.8            |            95.2            |
|      sofa       |           66.9            |            69.1            |
|      chair      |           20.8            |            32.3            |
|     cabinet     |           15.2            |            11.2            |
|  dining table   |           40.7            |            24.2            |
|     plants      |           29.5            |            26.3            |
|   tv cabinet    |           34.4            |            27.3            |
|  coffee table   |           56.4            |            64.9            |
|   side table    |           14.6            |            9.7             |
| air conditioner |           16.7            |            24.0            |
|     dresser     |           46.7            |            46.7            |
|      stool      |           17.6            |            30.8            |
|  refrigerator   |            0.0            |            16.7            |
|    painting     |           34.9            |            38.2            |
|     carpet      |           40.3            |            24.1            |
|       tv        |           16.0            |            18.0            |

</div>

### Result Visualizations

<div align="center">

|                                                            Layout Estimation                                                            |                                                          Object Detection                                                          |                                                       Zero-shot Reconstruction                                                        |
| :-------------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------: |
|                                                  ![Structured3D](./figures/stru3d.jpg)                                                  |                                                 ![ScanNet](./figures/scannet.jpg)                                                  |                                                 ![Zero-shot](./figures/zeroshot.jpg)                                                  |
| [Structured3D Results](https://manycore-research-azure.kujiale.com/manycore-research/SpatialLM/supplementary/visualization_layout.html) | [ScanNet Results](https://manycore-research-azure.kujiale.com/manycore-research/SpatialLM/supplementary/visualization_object.html) | [Zeroshot Results](https://manycore-research-azure.kujiale.com/manycore-research/SpatialLM/supplementary/visualization_zeroshot.html) |

</div>

## License

SpatialLM-Llama-1B is derived from Llama3.2-1B-Instruct, which is licensed under the Llama3.2 license.
SpatialLM-Qwen-0.5B is derived from the Qwen-2.5 series, originally licensed under the Apache 2.0 License.

SpatialLM1.0 are built upon the SceneScript point cloud encoder, licensed under the CC-BY-NC-4.0 License. TorchSparse, utilized in this project, is licensed under the MIT License.

SpatialLM1.1 are built upon Sonata point cloud encoder, model weight is licensed under the CC-BY-NC-4.0 License. Code built on Pointcept is licensed under the Apache 2.0 License.

## Citation

If you find this work useful, please consider citing:

```bibtex
@article{SpatialLM,
    title         = {SpatialLM: Training Large Language Models for Structured Indoor Modeling},
    author        = {Mao, Yongsen and Zhong, Junhao and Fang, Chuan and Zheng, Jia and Tang, Rui and Zhu, Hao and Tan, Ping and Zhou, Zihan},
    journal       = {arXiv preprint},
    year          = {2025},
    eprint        = {2506.07491},
    archivePrefix = {arXiv},
    primaryClass  = {cs.CV}
}
```

## Acknowledgements

We would like to thank the following projects that made this work possible:

[Llama3.2](https://github.com/meta-llama) | [Qwen2.5](https://github.com/QwenLM/Qwen2.5) | [Transformers](https://github.com/huggingface/transformers) | [SceneScript](https://github.com/facebookresearch/scenescript) | [TorchSparse](https://github.com/mit-han-lab/torchsparse) | [Sonata](https://xywu.me/sonata/) | [Pointcept](https://github.com/Pointcept/Pointcept)
