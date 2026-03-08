# ROS 2 distro IDL difftest

This directory provides a systematic way to compare **ROS IDL definitions** (.msg / .srv / .action) across ROS 2 distros and to inform cross-distro interoperability (e.g. Zenoh bridging, mixed RMW).

## Goals

1. **Quantify differences**: See how core message definitions change across distro branches (added/removed/changed fields, type changes).
2. **Reproducible**: Scripts fetch specified distro branches into local directories for diffing and automated analysis.
3. **Readable reports**: Generate human-readable Markdown reports and machine-friendly JSON for downstream tooling and compatibility design.

## Distro timeline and progression

Comparison is **progressive by release order**: each distro is compared only to the next (e.g. humble → jazzy → rolling), not pairwise. That matches how distros evolve over time.

| Distro               | Release Date      | EOL Date         | Support | Ubuntu         |
| -------------------- | ----------------- | ---------------- | ------- | -------------- |
| **Kilted** Kaiju      | May 23, 2025      | **Dec 2026**     | LTS     | 24.04 (Noble)  |
| **Jazzy** Jalisco     | May 23, 2024      | **May 2029**     | LTS     | 24.04 (Noble)  |
| Iron Irwini           | May 23, 2023      | November 2024    | —       | 22.04 (Jammy)  |
| **Humble** Hawksbill  | May 23, 2022      | **May 2027**     | LTS     | 22.04 (Jammy)  |
| Galactic Geochelone   | May 23, 2021      | December 9, 2022 | —       | 20.04 (Focal)  |
| Foxy Fitzroy          | June 5, 2020      | June 20, 2023    | —       | 20.04 (Focal)  |
| Eloquent … / Dashing … / older | 2017–2019 | various | — | 16.04 / 18.04 |

Chronological order used in this tool: **foxy → humble → iron → jazzy → kilted → rolling**. **Foxy** (LTS before Humble) is included as the most-used pre-Humble distro. Scripts use **foxy / humble / jazzy / rolling** by default in `config.yaml`; add or remove distros there as needed.

## Core message repos and branches

| Repo                     | Description                          | Typical packages / interfaces |
|--------------------------|--------------------------------------|-------------------------------|
| ros2/common_interfaces   | Common message and service definitions | std_msgs, geometry_msgs, sensor_msgs, nav_msgs, trajectory_msgs, etc. |
| ros2/rcl_interfaces      | Interfaces for ROS client libraries  | Parameters, logging, lifecycle |
| ros2/geometry2           | Transform library and messages       | tf2_msgs, tf2_geometry_msgs |
| ros-controls/control_msgs| Robot control messages and actions   | Joint/cartesian trajectories, controller setpoints |

Each repo has distro-named branches on GitHub (e.g. `humble`, `jazzy`, `rolling`). Scripts clone each branch into a separate directory.

## Layout

```
ros_distro_idl_difftest/
├── README.md                 # This file
├── .gitignore                # Ignore cloned repos/
├── config.yaml               # Repo list, distro list, Git URLs
├── scripts/
│   ├── fetch_repos.sh        # Fetch each repo per distro into repos/<repo>/<distro>
│   ├── fetch_repos.py
│   ├── analyze_idl.sh
│   └── analyze_idl.py        # Parse .msg/.srv/.action, compare across distros, emit reports
├── repos/                    # Cloned trees (gitignored)
│   ├── common_interfaces/
│   │   ├── humble/
│   │   ├── jazzy/
│   │   └── rolling/
│   ├── rcl_interfaces/
│   │   └── ...
│   └── ...
└── reports/                  # Generated reports (optional to commit)
    ├── idl_diff_report.md
    └── idl_diff_report.json
```

## Usage

### 1. Fetch distro branches

```bash
cd rust/tools/ros_distro_idl_difftest
./scripts/fetch_repos.sh
```

- Reads repo and distro list from `config.yaml`.
- Clones each repo’s distro branch into `repos/<repo_name>/<distro>/`.
- If a directory already exists and is a git repo, use `--refresh` to run `git fetch` + `git checkout` instead of skipping.

### 2. Analyze IDL and generate reports

```bash
./scripts/analyze_idl.py --repos-dir repos --out-dir reports
```

- Scans all `.msg`, `.srv`, `.action` under `repos/<repo>/<distro>/`.
- Aligns interfaces by package + kind + name across distros and compares fields (types, names, constants).
- Writes:
  - `reports/idl_diff_report.md`: Summary and per-interface table.
  - `reports/idl_diff_report.json`: Structured output for tooling.

### 3. Restrict to specific distros or repos

- Edit `distros` or `repos` in `config.yaml`. Both `fetch_repos.sh` and `analyze_idl.py` respect the config and existing `repos/` contents.

## Report contents

The report is **one large Markdown table**:

- **Header row**: `Interface` | `<distro1>` | `<distro2>` | … | `Changes (progressive)`
- **One row per interface** (e.g. `common_interfaces/std_msgs/msg/Header`): each distro column shows `✓ (n)` if present (n = field count) or `—` if missing; the last column summarizes **progressive** changes (e.g. `humble→jazzy: +field_x; jazzy→rolling: type y→z`).
- **Summary** above the table: repos, distro order, total interfaces, and per-repo counts (only in some distros, with progressive diff).

## Relation to zenoh_cross_ros

This experiment supports the [zenoh_cross_ros](../zenoh_cross_ros/README.md) “different IDL bridging and translation” work: knowing real differences in core interfaces across distros helps design canonical formats and translation layers and to prioritize which message types to support in cross-version setups.
