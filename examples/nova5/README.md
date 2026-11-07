# Nova5 RGB pick and return

Robonix Pilot observes the wrist RGB camera and selects `nova5_pick_return`
for one cycle: grasp the configured black box, lift it 30 mm, put it back,
and return the arm home. The skill uses fresh images for localization and
outcome checks. Arm motion uses the Nova5 SDK at 50 mm/s; the PGI gripper
uses Modbus RTU. The measured 165 mm flange-to-tip offset is applied in
software. No depth image is used.

## Hardware and external dependencies

This example requires separately supplied calibration code and a PGI driver.
They are local projects from the tested installation, not packages distributed
by Robonix. Obtain them from the example contributor before setting up another
installation. A fresh clone alone cannot run the cycle.

Required hardware is a Nova5, an Orbbec RGB wrist camera, a PGI gripper, and a
fixed checkerboard. The tested setup uses a Gemini 336L, a 25 mm checkerboard,
a 165 mm TCP offset, and a box measured at 50 mm tall. Keep the robot base,
camera mount, and board fixed. Recalibrate after changing their mounting.

Supply these paths in `settings.json`:

- `calibration_root`: the external project containing `handeye/`, `config.json`,
  and its calibration files. The scripts import `handeye.camera`, `cli`,
  `geometry`, `robot`, `test_motion`, and `workflow`, including their internal
  dependencies. Provide the complete tested package, not just calibration JSON.
  Its configuration must select the camera and robot address, locate the
  official SDK, and reference matching intrinsics, hand-eye, and plane results.
- `camera_python`: that project's Python environment with NumPy, SciPy, OpenCV,
  and `pyorbbecsdk` (the `pyorbbecsdk2` distribution). The tested camera environment
  uses Python 3.12 and `pyorbbecsdk2` 2.1.3. Follow the camera SDK's USB permission
  setup on the target machine.
- `pgi_module`: the external directory containing `pgi_hand/protocol.py` and
  `pgi_hand/motion.py`. `hand_python` must have pyserial and the Robonix Python
  dependencies available. This driver is separate from the manufacturer's SDK;
  a generic serial driver cannot replace its state and command checks.
- `runtime_python`: a Python environment with the Robonix dependencies,
  `grpcio-tools`, NumPy, SciPy, and `opencv-python-headless`. It can also serve as
  `hand_python` when pyserial is installed. The tested runtime uses Python 3.11,
  NumPy 2.4.6, SciPy 1.17.1, OpenCV 5.0.0.93, grpcio 1.80.0, and grpcio-tools 1.76.0.
- `generated_proto`: the example's generated `proto_gen` directory shown in
  `settings.example.json`. `rbnx boot` builds it from the package manifests.
- `atlas_binary`: a built Robonix Atlas executable. The tested hardware adapter
  uses the [official Dobot V4 SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V4)
  at revision `55ec1ec82201aaf2e6d54aa47b6272bbaa14c815`; supply its path through
  the calibration project's `robot.sdk_path` configuration.

Copy `settings.example.json` to `settings.json` and replace every placeholder.
Paths are absolute or relative to the Robonix repository root. Set the serial
port, measured box height, allowed XY range in the robot base frame, and a
validated home flange pose in millimetres/degrees. The XY range and home pose
are deliberately unset in the template. Local settings and calibration files
are excluded from version control.

## Run

Build/install Robonix as described in the repository README (`make install`).
The API endpoint must support `gpt-6-astra`, image input, tool calls, and
`high` reasoning effort. The example uses the API from Pilot and from its
visual checks.

```bash
cd examples/nova5
cp settings.example.json settings.json  # first setup only; edit before boot
export VLM_BASE_URL='https://your-openai-compatible-endpoint/v1'
read -rsp 'API key: ' VLM_API_KEY; echo
export VLM_API_KEY
./boot.sh
```

In another terminal, from the Robonix repository root:

```bash
ROBONIX_HOME="${ROBONIX_HOME:-$PWD/.artifacts/rbnx-config}" ./target/debug/rbnx ask 'Observe the black box on the checkerboard with the Nova5 wrist camera. If it is clearly visible, call nova5_pick_return to pick it up, put it back at the observed position, and return the arm home. Execute one cycle, wait for completion, and report the result.'
```

Before starting, the arm must be enabled and idle in TCP control mode with
zero User0/Tool0 offsets. The initialized gripper must report initialization 1,
idle 1, IO mode 0, force 20%, and speed 1%, with equal target and actual opening.
The TCP must be at least 200 mm above the calibrated table. Check that the
camera view and gripper path are clear. The routine is limited to the configured
box and workspace; its placeholder URDF does not supply arm kinematics.

## Checks and recovery

With the external environments configured and package code generation complete,
run the offline tests from the repository root:

```bash
PYTHONPATH="$PWD/pylib/robonix-api:$PWD/examples/nova5/skills/pick_return/rbnx-build/codegen/proto_gen" \
  /path/to/runtime/bin/python -m unittest discover -s examples/nova5 -p 'test_*.py'
```

The complete grasp, replace, and home cycle passed on the configured hardware
on 2026-11-07. A clean-clone hardware installation has not been validated.
Put a demonstration video directly in `examples/nova5/`.

Verify TCP ports 29999 and 30004 on the configured robot address and access to
the camera and serial device before booting. A DHCP controller needs a DHCP
server; a direct cable and a static PC address alone do not provide one.
Temporary USB permissions can disappear after reconnecting the device.

A failed or canceled cycle locks further runs. Inspect its log and the actual
robot/gripper state before restarting `boot.sh`. Restarting does not move the
arm home or initialize the gripper. Do not start another cycle from a low grasp
position or while holding an object.

Traces, snapshots, and skill logs stay in the ignored `.artifacts/` directory;
Robonix build and boot files are also ignored. The skill refuses new cycles
after 100 run records accumulate. Archive needed evidence and remove old
artifacts only while the example is stopped. Do not include logs, images from
test runs, credentials, or generated files in a PR.
