# Digital Twin of a Dual-Arm Satellite Proximity-Operations Testbed

Undergraduate Research Experience (CDE2605R UREx), National University of Singapore.

Two robot arms in a cleanroom each carry a satellite mockup, a **chaser** and a **target** (called *debris* in the code). By moving both arms, the testbed reproduces the *relative* 6-DoF motion between two spacecraft during close-proximity operations, such as approach, docking alignment and inspection fly-arounds. The motion is repeatable, so vision-based navigation algorithms can be tested on the ground.

The arms are two Seeed reBot B601-RS. The chaser arm hangs upside-down from the carriage of a 1.25 m overhead linear rail, which provides the along-track motion; the arm handles the radial offset and height. The target arm holds its mockup in one fixed pose.

## Repository layout

```
UnityProject/                     Unity 6 project (the 3D viewer)
  Assets/Scenes/SampleScene.unity   the scene: both arms, HIL table and overhead frame, satellite mockups
  Assets/Scripts/TestbedTwin.cs     ZeroMQ client that drives the arms and reports back
  Assets/Table/HIL_Table.fbx        table, frame and rail actuator from the SolidWorks assembly HIL_Main_Robotics
  Assets/Editor/TestbedLook.cs      Tools > Testbed > Apply CAD Look: recreates the CAD materials for the scenery
  Assets/RS/, Assets/RS_Debris/     vendor URDFs and meshes for the chaser (with rail) and target arms
python/
  proximity_ops_twin.ipynb          the analysis, as a notebook
  proximity_ops_twin.py             the same analysis as a plain script
  hardware_bridge.py                sends the same joint commands to the physical arms
  trajectory_orbit_lvlh.csv         reference trajectory (600 s approach, 0.1 s steps)
```

## Test-bed layout

Lab frame: right-handed, Z up, metres, origin at the centre of the tabletop's top surface, X along the table length (the rail direction). Unity (x, y, z) = (−Y, Z, X). Values come from the CAD assembly; replace them with measurements once the rig is built.

| Item | Lab frame | Unity Transform |
|---|---|---|
| Tabletop | 1.39 × 0.74 m, top at z = 0 | `HIL_Table` local position (0, 0.005, 0) inside `TestBed_Scenery` |
| Chaser arm (`ReBot_Chaser_Rail`) | hangs upside-down under the rail carriage; carriage at stroke 0 is x = −0.795, arm mount face z = 0.773 | Position (0, 0.793, −0.795), Rotation (0, 0, 180) |
| Target arm (`ReBot_Debris`) | base on the tabletop at (0.55, −0.29, 0), facing +Y | Position (0.29, 0, 0.55), Rotation (0, −90, 0) |
| Target satellite centre | (0.64, 0, 0.35) | |

- Stroke 0 is assumed to be the −X (drawer-box) end of the rail; the home end is not confirmed yet.
- The chaser keeps its satellite 0.23 m ahead of the carriage. At the start the carriage is 10 mm off its end stop, and the gap grows smoothly from 0.13 m to 0.23 m over the first 150 s.
- The chaser mock-up is mounted turned 180° about the gripper axis. Holding it upright from an upside-down arm would otherwise need joint 6 at ±180°, just outside its ±3.14 rad limit.
- In `TestbedTwin`, *Carriage Mesh* is the CAD carriage (`BODY2_E-MTHE17…`, parked at stroke 0); it is attached to the rail carriage on Play so it slides with the rail.

## Why Unity + Python

The project splits into two jobs with different needs, so each job uses the tool that suits it best.

**Unity does the visualisation.** Unity imports the vendor URDF directly and renders the arm meshes in 3D in real time. Python plotting tools are poor at animating a jointed 3D arm. Unity also opens a later option: rendering synthetic camera views of the mockups for pose-estimation work.

**Python does the analysis.** Python has mature libraries for this (NumPy and SciPy, with Pinocchio if speed becomes an issue). Unity is only a viewer. All kinematic results come from Python, so the analysis never depends on Unity.

**ZeroMQ connects the two.** It is a lightweight messaging library with bindings for both languages (`pyzmq` and NetMQ).
- **No ROS install.** The ROS–Unity bridge needs ROS 2. ZeroMQ needs only a Python package and a few DLLs.
- **Loose coupling.** It uses publish/subscribe. Python publishes joint commands, and any number of listeners (Unity, the hardware bridge) follow the latest one. Either side can start, stop or crash without breaking the other.
- **Simple protocol.** Each message is one line of JSON, in radians and metres. It is easy to inspect and to extend.

## What the Python side does

1. Loads the reference trajectory (chaser position relative to the target, in the target's LVLH frame).
2. Trims it to start at 1.3 m range (the rail stroke can't cover the full 1.5 m with the target on the 1.39 m × 0.74 m table), maps it onto the table and places the two robots.
3. Writes Unity-ready URDFs for the target arm and the chaser arm on its rail.
4. Solves inverse kinematics for every remaining sample (the rail position is set analytically, then a bounded least-squares solve gives the six arm joints).
5. Streams the joint commands to Unity and records what Unity reports back.
6. Checks the tracking error and that the separation reaches the 30 cm threshold.

## Requirements

- **Unity 6000.5.10f1** (install it through Unity Hub). NetMQ is included in `Assets/Packages`; Unity downloads the URDF Importer package from GitHub the first time the project opens, so the first open needs internet access.
- **Python 3.10 or newer** with `numpy pandas scipy matplotlib pyzmq`.

Set up Python once, from the repository root:

```bash
python3 -m venv .venv
.venv/bin/pip install numpy pandas scipy matplotlib pyzmq
```

## Running the simulation

1. **Open the project.** In Unity Hub, choose *Add → Add project from disk* and select the `UnityProject` folder.
2. **Open the scene.** In the Project window, double-click `Assets/Scenes/SampleScene`. On first open Unity shows an empty *Untitled* scene, because the last-opened scene is not stored in the repository.
3. **Turn on Run In Background (once).** *Edit → Project Settings → Player → Resolution and Presentation → Run In Background.* Without it, Unity pauses whenever its window loses focus, and the arms stop following.
4. **Press Play.** The Console should show `TestbedTwin: SUB tcp://localhost:5556, PUB tcp://localhost:5557`.
5. **Run the script** from the repository root:

   ```bash
   .venv/bin/python python/proximity_ops_twin.py
   ```

The script solves the IK (about 10 s), waits for Unity, eases the arms into the start pose and checks three test poses, then plays the approach. When it finishes, it prints the tracking error and the final separation, and opens the plots.

A good run looks like this:

```
IK acceptance (< 1 mm, < 0.1°, > 0.05 rad from limits): PASS
...
rail 0.01: chaser err    0.0 mm | debris err    0.0 mm
...
streamed 4902 samples in 51.0 s, received 1120 state messages
tracking error (Unity vs reference): mean 2.89 mm, max 21.87 mm
final separation: 30.00 cm (reference 30.00 cm)
threshold ≤ 30 cm reached at t = 600.0 s (CSV time)
```

Times are CSV times, so the run starts at t = 109.9 s. At 10× speed the tracking error has a ~2.5 mm baseline (the drives lag the targets) and spikes of ~20 mm whenever Unity stalls for a moment; keep the Unity window visible, or use `--speed 1`, for a cleaner run.

Test-pose errors of a few millimetres mean the joint conventions in Unity match Python. Errors of centimetres mean a joint is mirrored.

### Options

| Option | Effect |
|---|---|
| `--speed N` | Playback speed-up. Default `10` (the 600 s approach in 60 s); `1` is real time. Above about 50×, Unity may lag behind. |
| `--offline` | Only run the analysis and IK (no Unity needed). |
| `--no-plots` | Do not open plot windows at the end. |

The notebook runs the same steps. Run the cells top to bottom; sections 1–5 need no Unity, section 6 needs Unity in Play mode. Run `link.close()` (the last cell) before re-running section 6.

### Outputs

Each run writes to `python/`:
- `joint_trajectory.csv`: rail position and joint angles of both arms for every sample.
- `unity_run_log.csv`: what Unity measured (satellite centres and joint angles).

The script also rewrites the two URDFs in `UnityProject/Assets/RS`. The robots already in the scene are not affected.

## Messages

Python binds both sockets; Unity and the hardware bridge connect, so either side can be started first.

| Port | Direction | Topic | Content |
|---|---|---|---|
| 5556 | Python → listeners | `cmd` | `{"seq", "t", "chaser": [rail, q1…q6], "debris": [q1…q6], "ref_c", "ref_d"}` |
| 5557 | Unity → Python | `state` | measured satellite centres (lab frame), joint angles and robot base positions |

Joint angles are in radians, the rail in metres from its start. Python works in a right-handed lab frame (Z up, origin at the centre of the table top); Unity's left-handed frame is used only for rendering.

## Driving the physical arms

> **Not yet tested on hardware.** The bridge has only been tested in `--dry-run` mode. Check joint zeros and directions with small single-joint moves before the first full run.

`python/hardware_bridge.py` listens to the same `cmd` stream as Unity, so the real arms and the twin follow the same commands at the same time.

```
proximity_ops_twin.py ──cmd──┬──> Unity (viewer)
                             └──> hardware_bridge.py ──CAN──> chaser arm, target arm, rail
```

### Setup (Linux PC wired to the arms)

The RS motors use SocketCAN, so the bridge must run on Linux (Ubuntu recommended). Unity and the twin script can stay on another computer.

1. Install the vendor library:

   ```bash
   git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
   cd reBotArm_control_py && uv sync
   ```

2. Give each arm its own CAN bus (both arms use motor IDs 1–6). In the library's `config/` folder, copy `rebotarm_rs.yaml` to `rebotarm_rs_chaser.yaml` (`channel: can0`) and `rebotarm_rs_debris.yaml` (`channel: can1`).
3. Bring the buses up:

   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can1 up type can bitrate 1000000
   ```

4. Start the bridge, then the twin script:

   ```bash
   python hardware_bridge.py --host <IP of the computer running the twin script> \
       --vendor-dir ~/reBotArm_control_py
   ```

Use `--dry-run` to test without hardware, `--arms chaser` or `--arms debris` to drive one arm, and `--max-speed` (rad/s, default 0.3) to slow the arms down.

### Safety behaviour

- Targets are clipped to the URDF joint limits and rate-limited, so the first command or a jump becomes a slow move.
- Before enabling the motors, the bridge shows how far each arm will move and waits for Enter.
- If commands stop, the arms hold their last position.
- Ctrl+C stops following and holds. Motors are disabled only after you press Enter, because a disabled arm goes limp and falls. Support the arms by hand first.
- The test poses in the twin script also move the real arms.

### Rail

The rail has no vendor driver. `RailDriver.send()` in `hardware_bridge.py` is a placeholder: the bridge computes rail positions but does not move the rail. Fill it in with the call for your rail controller.

## Troubleshooting

| Problem | Fix |
|---|---|
| The scene is empty | Open `Assets/Scenes/SampleScene` (see step 2). |
| `no state from Unity` | Unity is not in Play mode, or the `TestbedTwin` component is disabled. |
| Large errors, or the arms stop mid-run | Turn on *Run In Background* (see step 3). |
| `Address already in use` | Another copy of the script, or the notebook, is still running. Stop it, or run `link.close()` in the notebook. |
| `ModuleNotFoundError: No module named 'zmq'` | Run with `.venv/bin/python`, or install the packages listed under Requirements. |
