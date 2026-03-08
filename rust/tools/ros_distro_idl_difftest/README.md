# ROS 2 distro IDL difftest

This directory provides a systematic way to compare **ROS IDL definitions** (.msg / .srv / .action) across ROS 2 distros and to inform cross-distro interoperability (e.g. Zenoh bridging, mixed RMW).

## Goals

1. **Quantify differences**: See how core message definitions change across distro branches (added/removed/changed fields, type changes).
2. **Reproducible**: Scripts fetch specified distro branches into local directories for diffing and automated analysis.
3. **Readable reports**: Generate human-readable Markdown reports and machine-friendly JSON for downstream tooling and compatibility design.

## Distros (most-used first)

| Distro   | Branch    | Note            |
|----------|-----------|-----------------|
| Humble   | `humble`  | LTS, widely used |
| Jazzy    | `jazzy`   | Current stable   |
| Rolling  | `rolling` | Development mainline |
| Iron     | `iron`    | Optional, EOL    |
| Kilted   | `kilted`  | Optional, newer  |

Scripts use **humble / jazzy / rolling** by default; extend via `config.yaml`.

## Core message repos and branches

| Repo                     | Description                          | Typical packages / interfaces |
|--------------------------|--------------------------------------|-------------------------------|
| ros2/common_interfaces   | Common message and service definitions | std_msgs, geometry_msgs, sensor_msgs, nav_msgs |
| ros2/rcl_interfaces      | Interfaces for ROS client libraries  | Parameters, logging, lifecycle |
| ros2/unique_identifier_msgs | UUID and identifier messages      | UUID.msg |

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
│   └── unique_identifier_msgs/
│       └── ...
└── reports/                  # Generated reports (optional to commit)
    ├── idl_diff_report_<timestamp>.md
    └── idl_diff_report_<timestamp>.json
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
  - `reports/idl_diff_report_<timestamp>.md`: Summary and per-interface diff.
  - `reports/idl_diff_report_<timestamp>.json`: Structured output for tooling.

### 3. Restrict to specific distros or repos

- Edit `distros` or `repos` in `config.yaml`. Both `fetch_repos.sh` and `analyze_idl.py` respect the config and existing `repos/` contents.

## Report contents

- **Per package**: Which distros have the package, which do not.
- **Per interface**: Field-level diffs for the same interface name across distros:
  - Added/removed fields
  - Type changes
  - Default/constant changes (if any)
- **Summary**: Counts per distro pair and per repo, including breaking changes (type or structure).

## Relation to zenoh_cross_ros

This experiment supports the [zenoh_cross_ros](../zenoh_cross_ros/README.md) “different IDL bridging and translation” work: knowing real differences in core interfaces across distros helps design canonical formats and translation layers and to prioritize which message types to support in cross-version setups.
