# # Proximity-ops digital twin — trajectory playback (Unity + ZeroMQ)
# **Digital Twin — Test Bed Team.** This notebook is the "brain" of the twin:
#
# 1. loads the reference trajectory (debris-centred LVLH, 0.1 s steps),
# 2. maps it into the lab/table frame and decides the robot layout,
# 3. writes Unity-ready URDFs for the two ReBot B601-RS arms (debris arm + chaser arm on a linear rail),
# 4. solves inverse kinematics for every sample,
# 5. streams joint commands to Unity over ZeroMQ (Unity side uses NetMQ) and records what Unity reports back,
# 6. checks tracking error and the ≤ 30 cm threshold.
#
# Script version of proximity_ops_twin.ipynb. Sections 1–5 need no Unity; section 6 needs Unity in Play mode.
#
#   pip install numpy pandas scipy matplotlib pyzmq
#   python proximity_ops_twin.py               # press Play in Unity first
#   python proximity_ops_twin.py --speed 1     # real time (600 s)
#   python proximity_ops_twin.py --offline     # sections 1–5 only, no Unity

# ## 1. Configuration — edit the two paths
# `RS_DIR` is the folder that contains `urdf/` and `meshes/` from the ReBot package
# (`rebot_b601_rs_urdf/rebotarm_bringup/description/RS`).

from pathlib import Path
import argparse, json, os, time, re
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from scipy.optimize import least_squares

ap = argparse.ArgumentParser(description="Proximity-ops digital twin: IK + trajectory playback to Unity")
ap.add_argument("--speed", type=float, default=10.0, help="playback speed-up (default 10: 600 s in 60 s)")
ap.add_argument("--offline", action="store_true", help="stop after IK (sections 1–5), don't connect to Unity")
ap.add_argument("--no-plots", action="store_true", help="don't open plot windows at the end")
args = ap.parse_args()
os.chdir(Path(__file__).resolve().parent)   # relative paths below are relative to this script

CSV_PATH = Path(r"trajectory_orbit_lvlh.csv")  # <-- edit
RS_DIR   = Path(r"../UnityProject/Assets/RS")  # <-- edit
SRC_URDF = RS_DIR / "urdf" / "ReBot_Arm_RS.urdf"

# ---- Test-bed layout (lab frame: right-handed, Z up, origin = centre of table top, metres) ----
# Values from the CAD assembly HIL_Main_Robotics; replace with measurements once the rig is built.
TABLE_L, TABLE_W = 1.390, 0.740          # table top, X along the 1390 mm length (= rail direction)
H        = 0.35                          # height of both satellite centres above the table (0.30 leaves the debris
                                         # arm no level pose from a base on the tabletop, and the hanging chaser less margin)
TCP      = 0.05                          # satellite centre sits 5 cm beyond the gripper tip, along the tool axis
SAT_SIZE = (0.10, 0.10)                  # (chaser, debris) cube edge for the Unity mock-ups

DEBRIS_CENTRE    = np.array([0.64, 0.00, H])   # debris is fixed here (origin of LVLH), near the +X end of the table
DEBRIS_BASE_POS  = np.array([0.55, -0.29, 0.0])  # on the tabletop; base plate (0.20 x 0.14 at this yaw): x 0.45…0.65, y −0.36…−0.22
DEBRIS_BASE_YAW  = np.deg2rad(90)              # debris arm faces +Y and holds the debris over the centreline

# Chaser arm hangs upside-down under the carriage of the overhead rail (MISUMI E-MTHE17-L10-1250).
# RAIL_BASE_POS = carriage at stroke 0, z = arm mount face (0.773) + RAIL_CARRIAGE_H; the rail frame is Rx(pi),
# so its +X is still lab +X and the arm base sits RAIL_CARRIAGE_H *below* it.
# Stroke 0 is assumed to be the -X (drawer-box) end; the motor/home end is not confirmed.
RAIL_BASE_POS    = np.array([-0.795, 0.00, 0.793])
RAIL_LENGTH      = 1.25
RAIL_CARRIAGE_H  = 0.02
CHASER_STANDOFF  = 0.23                          # satellite centre is kept 0.23 m ahead of the carriage (in X) ...
RAIL_END_MARGIN  = 0.01                          # ... except at the start: the carriage starts 10 mm off its end stop
STANDOFF_RAMP_S  = 150.0                         # and the standoff grows to CHASER_STANDOFF over this many (CSV) seconds.
                                                 # The hanging arm can't hold the satellite level
                                                 # directly below or behind the carriage; at the 0.18 m peak lateral offset
                                                 # only ~0.21–0.24 keeps joints 4/5 > 0.05 rad off their limits.
FRAME_BOX = (np.array([-1.08, -0.395, 0.0]), np.array([0.74, 0.395, 0.773]))  # inside the overhead frame, below the mount face

# The full 1.5 m approach needs ~1.3 m of rail plus the standoff, more than the stroke allows with the debris on the
# table, so the run starts at the first sample inside this range (option 1 of the layout decision). The CSV is not changed.
APPROACH_START_RANGE = 1.30

THRESHOLD = 0.30

# ## 2. Load and sanity-check the trajectory

traj = pd.read_csv(CSV_PATH)
n_csv = len(traj)
traj = traj[np.linalg.norm(traj[['x_m','y_m','z_m']].to_numpy(), axis=1) <= APPROACH_START_RANGE + 1e-9].reset_index(drop=True)
print(f"trimmed to range ≤ {APPROACH_START_RANGE} m: dropped the first {n_csv - len(traj)} of {n_csv} rows")
t   = traj.t_s.to_numpy()
r_l = traj[['x_m','y_m','z_m']].to_numpy()      # chaser position wrt debris, LVLH
v_l = traj[['vx_ms','vy_ms','vz_ms']].to_numpy()
rng = np.linalg.norm(r_l, axis=1)
print(f"{len(traj)} rows, t = {t[0]} … {t[-1]} s, dt = {np.diff(t).mean():.3f} s")
print(f"range: start {rng[0]:.3f} m, end {rng[-1]:.3f} m, min {rng.min():.3f} m")
print(f"x ∈ [{r_l[:,0].min():.3f}, {r_l[:,0].max():.3f}], y ∈ [{r_l[:,1].min():.3f}, {r_l[:,1].max():.3f}], z ∈ [{r_l[:,2].min():.3f}, {r_l[:,2].max():.3f}]")
# finite-difference check that the velocity columns are consistent with the positions
vfd = np.gradient(r_l, t, axis=0)
print("max |v_csv - v_fd| =", np.abs(vfd - v_l)[1:-1].max(), "m/s")

fig, ax = plt.subplots(1, 2, figsize=(11, 4))
ax[0].plot(r_l[:,1], r_l[:,0]); ax[0].plot(0, 0, 'k*', ms=12)
ax[0].set_xlabel('y (along-track) [m]'); ax[0].set_ylabel('x (radial) [m]'); ax[0].set_title('LVLH, debris at origin'); ax[0].axis('equal'); ax[0].grid()
ax[1].plot(t, rng); ax[1].axhline(THRESHOLD, color='r', ls='--', label='30 cm threshold')
ax[1].set_xlabel('t [s]'); ax[1].set_ylabel('range [m]'); ax[1].legend(); ax[1].grid()
plt.tight_layout()

# ## 3. Frames
# | Frame | Convention | Use |
# |---|---|---|
# | **LVLH** (debris-centred) | x radial, y along-track, z orbit normal; right-handed | the CSV |
# | **Lab** | right-handed, Z up, origin at centre of table top, X along the 2 m length | IK, everything in Python |
# | **Unity** | left-handed, Y up | rendering only |
#
# The trajectory is planar (z ≡ 0), so the orbital plane is laid flat on the table:
# lab X = LVLH y (approach axis along the table), lab Y = −LVLH x, lab Z = LVLH z (det = +1, a proper rotation).
# Chaser centre in the lab: **C(t) = D + R·r(t)**, with D the fixed debris centre.
#
# Lab → Unity uses the same convention as Unity's URDF Importer / ROS-TCP-Connector: Unity (x, y, z) = (−Y, Z, X).

R_L2W = np.array([[0, 1, 0],
                  [-1, 0, 0],
                  [0, 0, 1.]])
assert np.isclose(np.linalg.det(R_L2W), 1)

C_ref = DEBRIS_CENTRE + r_l @ R_L2W.T            # chaser centre, lab frame
D_ref = np.tile(DEBRIS_CENTRE, (len(t), 1))

def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def lab_to_unity(p):  p = np.asarray(p); return np.stack([-p[...,1], p[...,2], p[...,0]], -1)
def unity_to_lab(u):  u = np.asarray(u); return np.stack([u[...,2], -u[...,0], u[...,1]], -1)
def yaw_lab_to_unity_deg(yaw): return -np.rad2deg(yaw)   # rotation about lab +Z == rotation about Unity +Y with opposite sign

print("chaser centre X range:", C_ref[:,0].min().round(3), "…", C_ref[:,0].max().round(3), f" (frame X ∈ {FRAME_BOX[0][0]} … {FRAME_BOX[1][0]})")
print("chaser centre Y range:", C_ref[:,1].min().round(3), "…", C_ref[:,1].max().round(3), f" (frame Y ∈ ±{FRAME_BOX[1][1]})")
half = np.array(SAT_SIZE).max()/2      # whole satellite cube inside the frame volume
assert np.all(np.r_[C_ref, [DEBRIS_CENTRE]] - half > FRAME_BOX[0]) and np.all(np.r_[C_ref, [DEBRIS_CENTRE]] + half < FRAME_BOX[1])
foot = np.abs(Rz(DEBRIS_BASE_YAW)[:2,:2]) @ [0.07, 0.10]   # base-plate half extents in lab X, Y
assert np.all(np.abs(DEBRIS_BASE_POS[:2]) + foot <= [TABLE_L/2, TABLE_W/2]), "debris base plate is off the tabletop"

# ## 4. Unity-ready URDFs
# (rail_base has no visual: the CAD actuator in the scene is the rail body.)
# Two files are written next to `meshes/` in `RS_DIR`:
# * `ReBot_Debris.urdf` — the arm as delivered (mesh paths rewritten so Unity's importer finds them),
# * `ReBot_Chaser_Rail.urdf` — the same arm on a prismatic **rail_joint** (1.25 m stroke along +X).
#
# The ReBot reaches ~0.6–0.7 m and can only hold a level satellite between ~0.25 and 0.7 m from its base,
# so a single fixed arm cannot cover the 1.2 m of closing motion; the rail supplies the along-track motion
# and the arm handles the radial offset and height (this is the "linear rail / extra DoF" option in the brief).

src = SRC_URDF.read_text(encoding="utf-8")
# Unity's URDF Importer resolves package:// relative to the folder containing the .urdf,
# so the URDFs go in RS_DIR and meshes are referenced as package://meshes/...
fixed = src.replace("package://rebotarm_bringup/description/RS/", "package://")

debris_urdf = re.sub(r'<robot\s+name="[^"]*"', '<robot\n  name="ReBot_Debris"', fixed, count=1)
(RS_DIR / "ReBot_Debris.urdf").write_text(debris_urdf, encoding="utf-8")

rail_xml = f'''
  <material name="rail_grey"><color rgba="0.55 0.57 0.60 1" /></material>
  <link name="rail_base">
    <inertial><origin xyz="{RAIL_LENGTH/2} 0 0.01" rpy="0 0 0"/><mass value="5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.7" iyz="0" izz="0.7"/></inertial>
  </link>
  <link name="rail_carriage">
    <inertial><origin xyz="0 0 {RAIL_CARRIAGE_H/2}" rpy="0 0 0"/><mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <visual><origin xyz="0 0 {RAIL_CARRIAGE_H/2}" rpy="0 0 0"/>
      <geometry><box size="0.14 0.14 {RAIL_CARRIAGE_H}"/></geometry><material name="rail_grey"/></visual>
  </link>
  <joint name="rail_joint" type="prismatic">
    <origin xyz="0 0 0" rpy="0 0 0"/><parent link="rail_base"/><child link="rail_carriage"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="{RAIL_LENGTH}" effort="200" velocity="0.5"/>
  </joint>
  <joint name="rail_to_arm" type="fixed">
    <origin xyz="0 0 {RAIL_CARRIAGE_H}" rpy="0 0 0"/><parent link="rail_carriage"/><child link="base_link"/>
  </joint>
'''
chaser_urdf = re.sub(r'<robot\s+name="[^"]*"\s*>', '<robot\n  name="ReBot_Chaser_Rail">' + rail_xml, fixed, count=1)
(RS_DIR / "ReBot_Chaser_Rail.urdf").write_text(chaser_urdf, encoding="utf-8")

# sanity: every mesh referenced must exist on disk
missing = [m for m in set(re.findall(r'package://([^"]+)', fixed)) if not (RS_DIR / m).exists()]
print("written:", RS_DIR / "ReBot_Debris.urdf", "and", RS_DIR / "ReBot_Chaser_Rail.urdf")
print("missing meshes:", missing or "none")

# ## 5. Kinematics straight from the URDF, then IK for every sample
# Forward kinematics is built by chaining the URDF joint origins and axes (no ROS needed).
# IK: rail position is set analytically (keeps the satellite `CHASER_STANDOFF` ahead of the carriage, or parks the
# carriage `RAIL_END_MARGIN` off the end stop while the satellite is closer than that),
# then a bounded least-squares solve gives the 6 arm joints for the satellite **position and attitude**
# (the chaser is held LVLH-fixed with its +X face pointing at the debris; the debris arm holds one static pose).

def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp,   cp*sr,            cp*cr]])
def axang(a, q):
    a = a / np.linalg.norm(a); K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3) + np.sin(q)*K + (1-np.cos(q))*K@K
def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1.,0,0],[0,c,-s],[0,s,c]])
def hom(R=np.eye(3), p=np.zeros(3)): T = np.eye(4); T[:3,:3] = R; T[:3,3] = p; return T

class Arm:
    def __init__(self, urdf, base="base_link", tip="gripper_end"):
        J = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().findall('joint')}
        chain, l = [], tip
        while l != base: j = J[l]; chain.append(j); l = j.find('parent').get('link')
        self.joints = []
        for j in reversed(chain):
            o = j.find('origin'); ax = j.find('axis'); lim = j.find('limit')
            self.joints.append(dict(
                name=j.get('name'), type=j.get('type'),
                T=hom(rpy2R(*map(float, o.get('rpy').split())), np.array(list(map(float, o.get('xyz').split())))),
                axis=np.array(list(map(float, ax.get('xyz').split()))) if ax is not None else None,
                lim=(float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else None))
        act = [j for j in self.joints if j['type'] != 'fixed']
        self.names = [j['name'] for j in act]
        self.lo = np.array([j['lim'][0] for j in act]); self.hi = np.array([j['lim'][1] for j in act])
    def fk(self, q):
        T, i = np.eye(4), 0
        for j in self.joints:
            T = T @ j['T']
            if j['type'] != 'fixed': T = T @ hom(axang(j['axis'], q[i])); i += 1
        return T
    def ik(self, T_goal, q0, w_rot=0.05, w_reg=1e-3):
        p, R = T_goal[:3,3], T_goal[:3,:3]
        def res(q):
            T = self.fk(q)
            return np.r_[T[:3,3]-p, w_rot*(T[:3,0]-R[:,0]), w_rot*(T[:3,2]-R[:,2]), w_reg*(q-q0)]
        q0 = np.clip(q0, self.lo+1e-6, self.hi-1e-6)
        sol = least_squares(res, q0, bounds=(self.lo, self.hi), xtol=1e-12, ftol=1e-12)
        T = self.fk(sol.x)
        return sol.x, np.linalg.norm(T[:3,3]-p), np.degrees(np.arccos(np.clip((np.trace(T[:3,:3].T@R)-1)/2, -1, 1)))

arm = Arm(SRC_URDF)
print("arm joints:", arm.names)
print("limits [rad]:", list(zip(arm.lo, arm.hi)))
print("home tool pose (base frame):\n", arm.fk(np.zeros(6)).round(4))

# Desired satellite attitudes (lab frame). Tool frame of gripper_end: +X out of the gripper, +Z up at home.
R_CHASER = np.eye(3)                 # chaser +X face points along +X (toward the debris), Z up
R_DEBRIS = Rz(DEBRIS_BASE_YAW)       # debris simply held level by an arm facing +Y
T_tcp = hom(p=np.array([TCP, 0, 0])) # gripper_end -> satellite centre
# The hanging chaser's mock-up is mounted turned 180° about the tool axis: holding it Z-up with gripper Z-up would need
# joint6 = ±π, just outside its ±3.14 limit. The centre offset is the same, and the cube looks the same in Unity.
T_tcp_ch = hom(Rx(np.pi), np.array([TCP, 0, 0]))

def gripper_goal(centre, R_sat, T_tcp=T_tcp):     # satellite-centre pose -> gripper_end pose (lab)
    return hom(R_sat, centre) @ np.linalg.inv(T_tcp)

# --- debris arm: one static pose ---
T_deb_base = hom(Rz(DEBRIS_BASE_YAW), DEBRIS_BASE_POS)
seeds = [np.array([0, .8, 1.2, .4, 0, 0]), np.array([0, 1.5, 2.0, .5, 0, 0]), np.array([0, 1.0, 1.6, .6, 0, 0])]
rs = np.random.default_rng(0)
rand_seeds = list(rs.uniform(arm.lo, arm.hi, (150, 6)))   # multi-start for poses the upright seeds don't reach
def lim_margin(q): return np.minimum(q - arm.lo, arm.hi - q).min()
def ik_multi(goal, seeds):             # best solution; among exact ones prefer the one farthest from the joint limits
    sols = [arm.ik(goal, s) for s in seeds]
    return min(sols, key=lambda s: s[1] + 0.02*np.radians(s[2]) - (1e-4*min(lim_margin(s[0]), 0.3) if s[1] < 1e-4 and s[2] < 0.05 else 0))
q_deb, e_deb, a_deb = ik_multi(np.linalg.inv(T_deb_base) @ gripper_goal(DEBRIS_CENTRE, R_DEBRIS), seeds)
if e_deb > 1e-4: q_deb, e_deb, a_deb = ik_multi(np.linalg.inv(T_deb_base) @ gripper_goal(DEBRIS_CENTRE, R_DEBRIS), seeds + rand_seeds)
print(f"debris arm: q = {np.round(q_deb,4)}  pos err {e_deb*1e3:.3f} mm, att err {a_deb:.3f}°")

# --- chaser: rail (analytic) + arm (IK) for every sample ---
N = len(t)
# The standoff starts at whatever puts the carriage RAIL_END_MARGIN off its end stop and grows smoothly to CHASER_STANDOFF
# over STANDOFF_RAMP_S, so the arm reaches forward slowly instead of swinging while the carriage waits.
s0 = C_ref[0,0] - RAIL_BASE_POS[0] - RAIL_END_MARGIN
tau = np.clip((t - t[0]) / STANDOFF_RAMP_S, 0, 1)
standoff = s0 + (CHASER_STANDOFF - s0) * tau*tau*(3 - 2*tau)    # smoothstep: starts and ends with zero rate
rail = C_ref[:,0] - standoff - RAIL_BASE_POS[0]
print(f"rail {rail.min():.3f} … {rail.max():.3f} m (stroke 0 … {RAIL_LENGTH}), satellite ahead of carriage {standoff.min():.3f} … {standoff.max():.3f} m")
assert rail.min() >= 0 and rail.max() <= RAIL_LENGTH, (rail.min(), rail.max())
assert standoff.min() >= 0.10, "satellite too close to (or behind) the carriage for the hanging arm"
assert np.all(np.diff(rail) >= 0), "rail reverses direction"
T_RAIL = hom(Rx(np.pi), RAIL_BASE_POS)        # rail frame, upside-down: local +X = lab +X, local +Z = lab -Z
def chaser_base(rail_k): return T_RAIL @ hom(p=[rail_k, 0, RAIL_CARRIAGE_H])
Q_ch = np.zeros((N, 6)); E_ch = np.zeros(N); A_ch = np.zeros(N)
t0 = time.time()
for k in range(N):
    goal = np.linalg.inv(chaser_base(rail[k])) @ gripper_goal(C_ref[k], R_CHASER, T_tcp_ch)
    if k == 0: q, E_ch[0], A_ch[0] = ik_multi(goal, seeds + rand_seeds)   # hanging arm: the upright seeds rarely converge
    else:      q, E_ch[k], A_ch[k] = arm.ik(goal, q)
    Q_ch[k] = q
print(f"IK for {N} samples in {time.time()-t0:.1f} s")
print(f"chaser: max pos err {E_ch.max()*1e3:.3f} mm, max att err {A_ch.max():.3f}°")
qd = np.abs(np.diff(Q_ch, axis=0)).max(0) / np.diff(t).min()
print("max joint speed [rad/s]:", qd.round(4), " rail speed max [m/s]:", np.abs(np.diff(rail)/np.diff(t)).max().round(4))
margin = np.minimum(Q_ch - arm.lo, arm.hi - Q_ch).min(0)
print("min distance to joint limits [rad]:", margin.round(3), " debris:", np.minimum(q_deb - arm.lo, arm.hi - q_deb).round(3))

# Independent check: FK of the solved joints reproduces the reference separation
def chaser_centre_fk(rail_k, qk):
    return (chaser_base(rail_k) @ arm.fk(qk) @ T_tcp_ch)[:3,3]
deb_fk = (T_deb_base @ arm.fk(q_deb) @ T_tcp)[:3,3]
C_fk = np.array([chaser_centre_fk(rail[k], Q_ch[k]) for k in range(0, N, 10)])
sep_fk = np.linalg.norm(C_fk - deb_fk, axis=1)
print("max |sep_fk - range_ref| =", np.abs(sep_fk - rng[::10]).max()*1e3, "mm")

# Acceptance: sub-mm, sub-0.1°, clear of the joint limits
ok = E_ch.max() < 1e-3 and A_ch.max() < 0.1 and margin.min() > 0.05 and e_deb < 1e-3 and a_deb < 0.1 \
     and np.minimum(q_deb - arm.lo, arm.hi - q_deb).min() > 0.05
print("IK acceptance (< 1 mm, < 0.1°, > 0.05 rad from limits):", "PASS" if ok else "FAIL")

# Rough clearance check on the joint frames (not the meshes): nothing below the tabletop, chaser below the mount face
def frames(T_base, q):
    T, i, P = T_base.copy(), 0, []
    for j in arm.joints:
        T = T @ j['T']
        if j['type'] != 'fixed': T = T @ hom(axang(j['axis'], q[i])); i += 1
        P.append(T[:3,3])
    return np.array(P)
P_ch = np.vstack([frames(chaser_base(rail[k]), Q_ch[k]) for k in range(0, N, 50)])
P_de = frames(T_deb_base, q_deb)
print(f"joint frames: chaser z ∈ [{P_ch[:,2].min():.3f}, {P_ch[:,2].max():.3f}], debris z ∈ [{P_de[:,2].min():.3f}, {P_de[:,2].max():.3f}] (table top 0, mount face 0.773)")

fig, ax = plt.subplots(1, 2, figsize=(12, 4))
ax[0].plot(t, Q_ch); ax[0].legend(arm.names, fontsize=8); ax[0].set_title('chaser arm joints [rad]'); ax[0].set_xlabel('t [s]'); ax[0].grid()
ax[1].plot(t, rail); ax[1].set_title('rail position [m]'); ax[1].set_xlabel('t [s]'); ax[1].grid()
plt.tight_layout()

out = pd.DataFrame({'t_s': t, 'rail_joint': rail, **{f'chaser_{n}': Q_ch[:,i] for i, n in enumerate(arm.names)},
                    **{f'debris_{n}': np.full(N, q_deb[i]) for i, n in enumerate(arm.names)},
                    'ref_sep_m': rng})
out.to_csv("joint_trajectory.csv", index=False); print("saved joint_trajectory.csv")

# ## 5b. Numbers to type into Unity
# Place the two imported robots with these **Transform** values (the script in Unity checks them and reports back).

# The chaser is rotated 180° about lab X (= Unity Z), so its rotation is (0, 0, 180).
for name, pos, rot in [("ReBot_Chaser_Rail", RAIL_BASE_POS, (0, 0, 180)), ("ReBot_Debris", DEBRIS_BASE_POS, (0, yaw_lab_to_unity_deg(DEBRIS_BASE_YAW), 0))]:
    u = lab_to_unity(pos)
    print(f"{name:18s} Position ({u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f})   Rotation ({rot[0]:.1f}, {rot[1]:.1f}, {rot[2]:.1f})")
print(f"\nTestbedTwin inspector: Tcp = {TCP}, Chaser Size = {SAT_SIZE[0]}, Debris Size = {SAT_SIZE[1]}, "
      f"Table = {TABLE_W} x {TABLE_L}")

if args.offline:
    if not args.no_plots: plt.show()
    raise SystemExit(0)

# ## 6. ZeroMQ link to Unity
# * Python **binds** both sockets; Unity **connects** (so you can stop/start Play mode freely).
# * `tcp://*:5556` PUB, topic `cmd` → joint targets (radians, metres) + reference positions.
# * `tcp://*:5557` SUB, topic `state` ← what Unity actually did (satellite centres in lab frame, measured joints).
#
# Press **Play** in Unity, then run this script.

import zmq

class TwinLink:
    def __init__(self, cmd_port=5556, state_port=5557):
        self.ctx = zmq.Context.instance()
        self.pub = self.ctx.socket(zmq.PUB); self.pub.setsockopt(zmq.LINGER, 0); self.pub.bind(f"tcp://*:{cmd_port}")
        self.sub = self.ctx.socket(zmq.SUB); self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "state"); self.sub.bind(f"tcp://*:{state_port}")
        self.states = []
        self.last = None                             # last commanded (rail, q_chaser, q_debris)
    def send(self, seq, tk, rail_k, q_c, q_d, ref_c, ref_d):
        self.last = (float(rail_k), np.array(q_c, float), np.array(q_d, float))
        msg = dict(type="cmd", seq=int(seq), t=float(tk),
                   chaser=[float(rail_k)] + [float(x) for x in q_c], debris=[float(x) for x in q_d],
                   ref_c=[float(x) for x in ref_c], ref_d=[float(x) for x in ref_d])
        self.pub.send_multipart([b"cmd", json.dumps(msg).encode()])
    def poll(self):
        n = 0
        while self.sub.poll(0):
            _, payload = self.sub.recv_multipart(); s = json.loads(payload); s["rx"] = time.time()
            self.states.append(s); n += 1
        return n
    def wait_for_unity(self, timeout=60):
        # Only listen: Unity publishes its state even without commands. Sending the start pose here would make
        # the drives (and the real arms, via hardware_bridge.py) jump to it in one step.
        t_end = time.time() + timeout
        while time.time() < t_end:
            if self.poll():
                s = self.states[-1]
                if self.last is None:                # start the first move from where the robots are now
                    qc, qd = np.array(s.get("q_chaser") or [np.nan]*7, float), np.array(s.get("q_debris") or [np.nan]*6, float)
                    qc = np.where(np.isfinite(qc), qc, np.r_[rail[0], Q_ch[0]]); qd = np.where(np.isfinite(qd), qd, q_deb)
                    self.last = (float(qc[0]), qc[1:], qd)
                return s
            time.sleep(0.1)
        raise TimeoutError("no state from Unity — is it in Play mode and is TestbedTwin enabled?")
    def close(self): self.pub.close(); self.sub.close()

link = TwinLink()
print("waiting for Unity (press Play)…")
hello = link.wait_for_unity()
print("Unity connected. Reported bases (lab):", hello.get("chaser_base"), hello.get("debris_base"))
for nm, rep, exp in [("chaser", hello.get("chaser_base"), RAIL_BASE_POS), ("debris", hello.get("debris_base"), DEBRIS_BASE_POS)]:
    if rep is not None and np.linalg.norm(np.array(rep) - exp) > 0.005:
        print(f"⚠ {nm} robot is not where the notebook expects ({rep} vs {exp}) — fix its Transform in Unity")

# ### 6a. Joint-convention check (do this once after importing)
# Eases into the start pose, then two test poses; compares the satellite position Unity measures with Python FK.
# Errors of a few mm mean the joint signs/axes match; errors of centimetres mean a joint is mirrored.

MOVE_JOINT_SPEED, MOVE_RAIL_SPEED = 0.3, 0.05      # rad/s, m/s for moves between poses outside the trajectory

def move_to(rail_k, q_c, q_d=q_deb):
    """Ease from the last commanded pose to this one in joint space (cosine profile, peak speed ≤ ~1.6× the limits)."""
    r0, qc0, qd0 = link.last
    T = max(1.0, np.abs(q_c - qc0).max()/MOVE_JOINT_SPEED, np.abs(q_d - qd0).max()/MOVE_JOINT_SPEED,
            abs(rail_k - r0)/MOVE_RAIL_SPEED)
    t_start = time.time()
    while (u := (time.time() - t_start)/T) < 1:
        a = (1 - np.cos(np.pi*u))/2
        rk, qc, qd = r0 + a*(rail_k - r0), qc0 + a*(q_c - qc0), qd0 + a*(q_d - qd0)
        link.send(-2, 0, rk, qc, qd, chaser_centre_fk(rk, qc), deb_fk); link.poll(); time.sleep(0.02)

def hold(rail_k, q_c, seconds=3.0):
    move_to(rail_k, q_c)
    t_end = time.time() + seconds
    while time.time() < t_end:
        link.send(-2, 0, rail_k, q_c, q_deb, chaser_centre_fk(rail_k, q_c), deb_fk); link.poll(); time.sleep(0.05)
    s = link.states[-1]
    return np.array(s["chaser_c"]), np.array(s["debris_c"])

tests = [(rail[0], Q_ch[0]), (0.4, Q_ch[0] + np.array([0.3, 0.2, 0.3, 0.2, 0.3, 0.5])), (0.8, Q_ch[-1])]
for rk, qk in tests:
    qk = np.clip(qk, arm.lo, arm.hi)
    c_u, d_u = hold(rk, qk)
    print(f"rail {rk:.2f}: chaser err {np.linalg.norm(c_u - chaser_centre_fk(rk, qk))*1e3:6.1f} mm | "
          f"debris err {np.linalg.norm(d_u - deb_fk)*1e3:6.1f} mm")

# ### 6b. Play the trajectory
# `SPEED = 10` plays the 600 s approach in 60 s. Every 0.1 s sample is sent; Unity reports at ~30 Hz.

SPEED = args.speed
link.states.clear()
hold(rail[0], Q_ch[0], 2.0)                    # ease back to the start pose and settle
link.states.clear()
t_wall0 = time.time()
for k in range(N):
    link.send(k, t[k], rail[k], Q_ch[k], q_deb, C_ref[k], D_ref[k])
    link.poll()
    sleep = t_wall0 + (t[k] - t[0])/SPEED - time.time()   # t starts after the trimmed part
    if sleep > 0: time.sleep(sleep)
hold_end = time.time() + 2.0                   # keep sending the final pose while the drives settle
while time.time() < hold_end:
    link.send(N-1, t[-1], rail[-1], Q_ch[-1], q_deb, C_ref[-1], D_ref[-1]); link.poll(); time.sleep(0.05)
print(f"streamed {N} samples in {time.time()-t_wall0:.1f} s, received {len(link.states)} state messages")

# ## 7. Did the twin follow the trajectory?

S = pd.DataFrame([s for s in link.states if s.get("seq", -1) >= 0])
S = S.drop_duplicates("seq", keep="last").sort_values("seq")
k = S.seq.to_numpy()
c_meas = np.vstack(S.chaser_c); d_meas = np.vstack(S.debris_c)
sep_meas = np.linalg.norm(c_meas - d_meas, axis=1)
err = np.linalg.norm(c_meas - C_ref[k], axis=1)
print(f"tracking error (Unity vs reference): mean {err.mean()*1e3:.2f} mm, max {err.max()*1e3:.2f} mm")
print(f"final separation: {sep_meas[-1]*100:.2f} cm (reference {rng[-1]*100:.2f} cm)")
TOL = 0.002   # the reference ends exactly on 30.0 cm, so allow 2 mm of drive/IK error
hit = np.where(sep_meas <= THRESHOLD + TOL)[0]
print("threshold ≤ 30 cm reached at t =", f"{t[k[hit[0]]]:.1f} s (CSV time)" if len(hit) else "never")

fig, ax = plt.subplots(1, 3, figsize=(15, 4))
ax[0].plot(t, rng, 'k--', label='reference'); ax[0].plot(t[k], sep_meas, label='Unity'); ax[0].axhline(THRESHOLD, color='r', ls=':')
ax[0].set_title('separation [m]'); ax[0].legend(); ax[0].grid()
ax[1].plot(t[k], err*1e3); ax[1].set_title('chaser tracking error [mm]'); ax[1].grid()
ax[2].plot(C_ref[:,0], C_ref[:,1], 'k--', label='reference'); ax[2].plot(c_meas[:,0], c_meas[:,1], label='Unity')
ax[2].plot(*DEBRIS_CENTRE[:2], 'r*', ms=12); ax[2].set_title('top view, lab frame [m]'); ax[2].axis('equal'); ax[2].legend(); ax[2].grid()
for a in ax[:2]: a.set_xlabel('t [s]')
plt.tight_layout()
S.to_csv("unity_run_log.csv", index=False)

link.close()
if not args.no_plots: plt.show()
