# Deployment reference

[Quick start](../README.md)


Independent dual-arm VR teleoperation and episode recording for an already deployed G1D bridge. The left and right controllers move their corresponding arms; each trigger controls its Dex1 gripper. This repository contains the teleoperation application and its small local control helpers.

## Behavior

- A fresh right-A click starts takeover from the current measured arm pose. Release A before starting a new episode.
- A later right-A hold of one second ends the episode normally. Recording also ends after 60 seconds of measured arm motion.
- Normal completion returns the arms to the saved initial pose, opens both grippers, and holds. Body height stays unchanged.
- B, stale input, invalid feedback, or a controller fault stops the episode and requests hold. Fault recovery is explicit; a fault does not automatically open the grippers.
- The manager requires confirmed return and both open grippers before preparing the next episode. It never runs a second controller alongside the current one.

Video rendering and 50 Hz XR publication use separate workers. DDS feedback is read through passive latest-sample subscriptions. The deployed bridge remains the sole DDS command publisher.

## Repository layout

```text
src/g1d_vr_teleop/
  cli.py, __main__.py      Unified command-line entry points
  config.py               Workspace, state, dependency and interpreter paths
  control/                Session lifecycle, launch, return and gripper control
  input/                  XR input service and event validation
  feedback/               Passive latest-sample arm and gripper readers
  recording/              Recorder management, video, actions and interval trimming
  core/                   Pure control helpers and the public pause RPC
tests/                    Offline unit tests
config/                   Local configuration example
docs/                     Deployment reference
```

The source package contains code only. Mutable state and recordings live in the configured workspace, outside `src/`.

## Installation

Python 3.10 or later is required. From the repository root, install into the Python environment intended for this application:

```bash
python -m pip install -e '.[test]'
g1d-vr --help
```

`g1d-vr` and `python -m g1d_vr_teleop` are equivalent entry points. For example, `python -m g1d_vr_teleop run --execute` runs the same manager as `g1d-vr run --execute`. Use the module form to select a specific Python interpreter. Installation supplies the application dependencies; the robot SDK, bridge client, and IK/XR environment below remain deployment prerequisites.

## Runtime prerequisites

Run on the Linux robot computer with an existing, compatible protocol-v3 `ial_g1d` bridge in healthy READY state. Install its client package, Unitree SDK2 Python/Cyclone DDS, and the compatible `xr_teleoperate` numerical IK and TeleVuer environment. The application does not install or restart the bridge, lower the body, or perform standard startup.

Compatible upstream dependency revisions:

| Dependency | Revision |
| --- | --- |
| `unitreerobotics/unitree_sdk2_python` | `a7dff75ba6ab0895710db9e9a065309b989d7e7a` |
| `unitreerobotics/xr_teleoperate` | `5ad63c6b3fbd4ff0566d12e70871f900915af452` |

Use the existing robot environment when available. A generic `pin` wheel is not a substitute for an environment with `pinocchio.casadi`, the matching robot model/assets, CasADi, Meshcat, and the XR dependencies. Model caches and assets are not distributed here. `G1D_VR_URDF` must match the model used by IK under `G1D_VR_XR_ROOT/assets`; the example points at that same model. Do not reuse a model cache from a different URDF.

The three existing JPEG publishers must be available locally:

| View | Port | Resolution |
| --- | --- | --- |
| Head stereo | 55555 | 1280 × 480 |
| Left wrist | 55556 | 640 × 480 |
| Right wrist | 55557 | 640 × 480 |

Camera drivers, calibration, and grasp planning are outside this repository.

## Configuration

Copy `config/site.example.env` to `config/site.local.env` and set the paths for your installation. Load it in each terminal used to run the application:

```bash
set -a
. config/site.local.env
set +a
```

`src/g1d_vr_teleop/config.py` is the single source for installation paths. The workspace defaults to the working directory when a command starts; set `G1D_VR_WORKSPACE` to use a fixed workspace from any directory. All configured relative paths resolve against that workspace. Start both terminals from the same workspace, or export the same absolute `G1D_VR_WORKSPACE` in both.

| Setting | Default location in the workspace | Contents |
| --- | --- | --- |
| `G1D_VR_STATE_ROOT` | `state/` | Initial pose, owner locks, current metadata and control sessions |
| `G1D_VR_INPUT_ROOT` | `state/input/` | XR input status and its local public directory |
| `G1D_VR_RECORD_ROOT` | `records/` | Raw and trimmed episode recordings |

The launcher passes the resolved workspace and package import path to child processes. The IK process uses the upstream `xr_teleoperate/teleop` working directory for model assets without changing where application state is stored. No SSH account or password is needed by the application.

`G1D_VR_PYTHON` defaults to the current Python interpreter; the camera recorder defaults to the same interpreter. The control interpreter also requires OpenCV for final trimming; a separate camera interpreter does not remove that requirement. Set a separate camera interpreter only if the deployed recorder environment needs it. Certificates must be generated locally and trusted by the headset; certificate/key contents are never committed.

An editable installation shares this repository's `src/` with recorder processes. For wheel installations with separate control and camera interpreters, install this package in both environments; the application does not forward one environment's entire `site-packages` into the other.

## Starting a session

Before the first launch, obtain a completed standard-startup receipt from the existing robot deployment, then bind the initial pose for the current boot:

```bash
g1d-vr bind --startup-receipt state/startup-receipt.json --execute
```

This reads a completed READY pose; it does not perform startup or move the robot. Existing initial references for the current boot are preserved in `state/initial-pose.json` by default. The initial reference and all live state are local, ignored files.

Start the XR input service in one terminal:

```bash
g1d-vr input
```

Open `https://ROBOT_HOST:8012/?ws=wss://ROBOT_HOST:8012` on the headset, substituting the robot computer address. Enter immersive VR and verify that the overlay reports fresh controller input. A working video page alone does not establish control readiness.

Start the manager in another terminal:

```bash
g1d-vr run --execute
```

For a single episode use `g1d-vr session --execute`. These are alternative owners: do not launch them together. The manager prepares an episode and waits for a new A press. Without `--execute`, `run` reports its behavior without starting a controller; `session` inspects a launch plan and requires a healthy bridge and a current initial-pose reference.

The normal return trajectory is capped at 0.10 rad/s with 0.25 rad/s² acceleration; the 0.08 rad tracking-error check remains active. A measured return start may lie within the planning margin if it remains inside the URDF hard limits; the saved endpoint must still satisfy the full margin. The bounded path moves inward to that endpoint. Opening the grippers occurs only after the return endpoint has stabilized.

Two separate operator commands are available when no other controller owns the session:

```bash
g1d-vr restore --execute
g1d-vr recover --output state/recovery.json --execute
```

`restore` returns to the saved pose from a healthy READY state. `recover` is restricted to the supported stopped-policy watchdog state and writes a new receipt; it does not provide a general fault reset. These commands are separate operations, not a sequence to run routinely. Inspect their help before use.

## Recording

Each episode receives separate control and recording directories. Three MP4s, frame timestamps, and compressed DDS action/state are recorded. `raw/` includes diagnostic pre-roll. Only `task/` contains the delivered interval, trimmed from sustained measured arm motion until normal finish or interruption. No measured motion means no task clip. Timestamps share the robot's host monotonic clock; they are not hardware-synchronized exposure times.

The main outputs in `task/` are `head_stereo.mp4`, `left_wrist.mp4`, `right_wrist.mp4`, `frames.jsonl`, `action_state.jsonl.gz`, and `episode.json`. The control status separately reports return success or failure. A verified recording does not imply a successful return or grasp.

The manager starts the internal video and action recorders through `g1d_vr_teleop.recording.video` and `g1d_vr_teleop.recording.actions`. Normal session use does not require launching these modules separately.

## Offline verification

After installing the test extra, run from the repository root:

```bash
python -m unittest discover -s tests -t . -p 'test_*.py'
```

External IK/bridge compatibility tests are skipped when their optional deployment packages are absent.

The tests exercise input freshness, A/B semantics, feedback validation, controller ownership, initial anchoring, return logic, and recording boundaries using mocks. They do not authorize or perform hardware motion.

No recordings, live sessions, initial poses, credentials, certificates, private keys, model caches, bridge service, or full robot deployment are included.
