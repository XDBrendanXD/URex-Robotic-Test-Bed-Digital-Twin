# URex-Robotic-Test-Bed-Digital-Twin

Undergraduate Research Experience (CDE2605R UREx), National University of Singapore.

Two robot arms in a cleanroom each carry a satellite mockup, a **chaser** and a **target**. By moving both arms, the testbed reproduces the *relative* 6-DoF motion between two spacecraft during close-proximity operations, such as approach, docking alignment and inspection fly-arounds. The motion is repeatable, so vision-based navigation algorithms can be tested on the ground.

## Why Unity + Python

The project splits into two jobs with different needs, so each job uses the tool that suits it best.

**Unity does the visualisation.** Unity imports the vendor URDF directly, renders the arm meshes in 3D in real time. Python plotting tools are poor at animating a jointed 3D arm. Unity also opens a later option: rendering synthetic camera views of the mockups for pose-estimation work.

**Python does the analysis.**  Python has mature libraries for this (NumPy, `roboticstoolbox-python`, and Pinocchio if speed becomes an issue). Unity is only a viewer. All kinematic results come from Python, so the analysis never depends on Unity.

**ZeroMQ connects the two.** It is a lightweight messaging library with bindings for both languages (`pyzmq` and NetMQ).
- **No ROS install.** The ROS–Unity bridge needs ROS 2. ZeroMQ needs only a Python package and two DLLs.
- **Loose coupling.** It uses publish/subscribe. Python publishes joint angles, and Unity displays the latest ones. Either side can start, stop or crash without breaking the other.
- **Simple protocol.** Each message is a single line of JSON, `{"joints": [...]}`, in radians. It is easy to inspect and to extend to a second arm.

