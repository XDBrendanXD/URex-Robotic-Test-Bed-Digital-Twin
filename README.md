# Digital Twin of a Dual-Arm Satellite Proximity-Operations Testbed

Undergraduate Research Experience (CDE2605R UREx), National University of Singapore.

Two robot arms in a cleanroom each carry a satellite mockup, a **chaser** and a **target** (called *debris* in the code). By moving both arms, the testbed reproduces the *relative* 6-DoF motion between two spacecraft during close-proximity operations, such as approach, docking alignment and inspection fly-arounds. The motion is repeatable, so vision-based navigation algorithms can be tested on the ground.

The arms are two Seeed reBot B601-RS. The chaser arm rides on a 1.25 m linear rail, which provides the along-track motion; the arm handles the radial offset and height. The target arm holds its mockup in one fixed pose.

## Repository layout

```
UnityProject/                     Unity 6 project (the 3D viewer)
  Assets/Scenes/SampleScene.unity   the scene: both arms, table, satellite mockups
  Assets/Scripts/TestbedTwin.cs     ZeroMQ client that drives the arms and reports back
  Assets/RS/, Assets/RS_Debris/     vendor URDFs and meshes for the chaser (with rail) and target arms
python/
  proximity_ops_twin.ipynb          the analysis, as a notebook
  proximity_ops_twin.py             the same analysis as a plain script
  hardware_bridge.py                sends the same joint commands to the physical arms
  trajectory_orbit_lvlh.csv         reference trajectory (600 s approach, 0.1 s steps)
```

## Why Unity + Python

The project splits into two jobs with different needs, so each job uses the tool that suits it best.

**Unity does the visualisation.** Unity imports the vendor URDF directly and renders the arm meshes in 3D in real time. Python plotting tools are poor at animating a jointed 3D arm. Unity also opens a later option: rendering synthetic camera views of the mockups for pose-estimation work.

**Python does the analysis.** Python has mature libraries for this (NumPy and SciPy, with Pinocchio if speed becomes an issue). Unity is only a viewer. All kinematic results come from Python, so the analysis never depends on Unity.

**ZeroMQ connects the two.** It is a lightweight messaging library with bindings for both languages (`pyzmq` and NetMQ).
- **No ROS install.** The ROS–Unity bridge needs ROS 2. ZeroMQ needs only a Python package and a few DLLs.
- **Loose coupling.** It uses publish/subscribe. Python publishes joint commands, and any number of listeners (Unity, the hardware bridge) follow the latest one. Either side can start, stop or crash without breaking the other.
- **Simple protocol.** Each message is one line of JSON, in radians and metres. It is easy to inspect and to extend.
