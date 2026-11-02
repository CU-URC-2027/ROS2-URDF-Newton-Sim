# URC Rover Arm Simulation (Newton + URDF)

A physics simulation of our rover arm, used to check reach, poses and **motor
torques** before hardware is built. It runs in
[NVIDIA Newton](https://github.com/newton-physics/newton) (a GPU physics
engine built on MuJoCo) and generates a **URDF** robot description of the
arm.

The arm: a **base turntable** (yaw), **shoulder**, **elbow** and **wrist**
(pitch), and a **gripper yaw**. Link 1 = 1.0 m and link 2 = 0.84 m axis to
axis, gripper 0.15 m. It is the successor to the team's MATLAB arm sim and
uses the same coordinate conventions.

Everything about the arm (lengths, tubes, motors, masses, joint limits,
presets) lives in **one file, `arm_params.py`**.

---

## Contents

1. [What's in this folder](#1-whats-in-this-folder)
2. [Quick start (already set up)](#2-quick-start-already-set-up)
3. [Setup from scratch](#3-setup-from-scratch)
4. [Using the sim](#4-using-the-sim)
5. [How the sim works](#5-how-the-sim-works)
6. [Working with git](#6-working-with-git)
7. [Troubleshooting](#7-troubleshooting)
8. [Limits and placeholders](#8-limits-and-placeholders)

---

## 1. What's in this folder

| File | What it is |
|---|---|
| `arm_params.py` | **The file you edit.** Lengths, tube section and material, pivot offsets, joint limits / effort / speed, motors, extra masses, COM overrides, payload, home pose, preset targets. |
| `sim_arm.py` | The simulation and its side panel (sliders, presets, IK, recording, design editing). |
| `arm_model.py` | The arm's math: geometry, mass properties, forward/inverse kinematics, static torques. |
| `build_urdf.py` | Writes `urc_arm.urdf` from the params and prints masses, tube lengths and holding torques. |
| `make_video.py` | Turns a recording into an MP4 (the sim runs it automatically). |
| `params_io.py` | Writes design changes made in the sim back into `arm_params.py`. |
| `urc_arm.urdf` | Generated robot description (for other tools, e.g. ROS). Regenerate with `python build_urdf.py`; don't edit by hand. |
| `requirements.txt` | Python packages to install. |

---

## 2. Quick start (already set up)

```bash
cd ~/urc_arm_sim               # wherever this folder is
source .venv/bin/activate      # the prompt should start with (.venv)
python sim_arm.py
```

---

## 3. Setup from scratch

Written for someone new to Ubuntu. Tested on **Ubuntu 24.04**
(dual boot) with an NVIDIA GPU.

### 3.1 Ubuntu basics

* **Terminal:** press `Ctrl+Alt+T`. Everything below is typed there.
* **Paste into the terminal with `Ctrl+Shift+V`** (plain `Ctrl+V` doesn't work).
* **Run one block at a time.** Paste, press Enter, wait for the prompt (`$`)
  to come back. Lines starting with `#` are comments.
* **`sudo`** runs a command as administrator and asks for your login
  password. Nothing appears while you type it; that's normal.
* **`apt`** is Ubuntu's app store for the terminal:
  `sudo apt install <name>`. Answer `Y` if asked.
* **`~`** means your home folder (`/home/<you>`). `cd <folder>` moves into a
  folder, `cd ~` goes home, `ls` lists what's here, `pwd` prints where you
  are. **Most "file not found" errors mean you're in the wrong folder.**
* **The prompt shows where you are:** `you@pc:~/urc_arm_sim$`.

### 3.2 NVIDIA driver (for the GPU)

```bash
nvidia-smi
```

If it prints a table with your GPU and a driver version of **545 or
higher**, skip ahead. If it says "command not found":

```bash
sudo ubuntu-drivers install
sudo reboot
```

No GPU? The sim still runs on the CPU, just slower.

### 3.3 Get the code

```bash
sudo apt update
sudo apt install git python3-venv python3-pip
cd ~
git clone https://github.com/<your-team>/<repo>.git
cd <repo>/urc_arm_sim          # adjust to where this folder sits in the repo
```

### 3.4 Python environment and Newton

A **virtual environment** ("venv") is a private Python install for this
project, so its packages don't clash with the system (Ubuntu 24.04 blocks
installing packages into the system Python).

```bash
# libraries for the 3D viewer window
sudo apt install -y libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev

# create and activate the venv inside this folder
python3 -m venv .venv
source .venv/bin/activate      # prompt now starts with (.venv)

# install Newton, the viewer and matplotlib
pip install -r requirements.txt
```

**Activate the venv every time you open a new terminal for the sim**
(`source .venv/bin/activate` from this folder). To make that one word, add
an alias (change the path to yours):

```bash
echo "alias armsim='cd ~/<repo>/urc_arm_sim && source .venv/bin/activate'" >> ~/.bashrc
source ~/.bashrc
```

Now `armsim` in any terminal takes you there and activates it. Don't put
`source .venv/bin/activate` itself in `~/.bashrc`: it would activate in
every terminal.

Test Newton:

```bash
python -m newton.examples basic_pendulum
```

A window with a swinging pendulum should open.

### 3.5 Video tools (for recordings)

```bash
sudo apt install ffmpeg vlc
```

`ffmpeg` encodes the MP4s; VLC plays them (Ubuntu has no video player by
default).

### 3.6 Check the sim

```bash
python sim_arm.py --viewer null --test --num-frames 300
```

It should end with `OK: sim holding torques match the static model`. Then
open the real thing:

```bash
python sim_arm.py
```

---

## 4. Using the sim

```bash
armsim            # or: cd to this folder && source .venv/bin/activate
python sim_arm.py
```

### 4.1 The 3D view

The arm stands on a grey box (the rover body). Blue boxes are the link
tubes, orange cylinders are motors, grey boxes are other masses, red rods
mark the pitch axes and the green ball is the TCP (gripper tip).

| Control | Action |
|---|---|
| Middle drag | Orbit |
| Shift + middle drag | Pan |
| Scroll | Zoom |
| WASD / QE | Move the camera / up-down |
| F | Frame the arm |
| Right drag | Grab and push a body |
| Space | Pause / resume |
| H | Hide / show the panels |

### 4.2 Side panel

**Every slider has a number box next to it:** drag the slider, or click the
box, type a value and press **Enter**.

| Section | What it does |
|---|---|
| **Status** | What the sim is doing, or why something failed (e.g. "out of reach"). |
| **Joints** | Joint angles in **relative degrees** (what each motor turns). The arm moves there smoothly, limited by each joint's speed. |
| **Presets** | Buttons for saved poses. Edit `PRESETS` in `arm_params.py`. |
| **TCP target / IK** | Type a gripper-tip position (m) and pitch (deg; −90 = pointing down, 0 = level) and press **Solve IK**. The turntable turns to face the target. **Elbow up** picks between the two arm solutions. |
| **Record move** | Records a move from a start pose to an end pose (4.3). |
| **Design** | Move pivots, motors and masses live and save them (4.4). |
| **Readout** | TCP position and, for every motor: torque now, static (holding) torque, and effort limit. **SAT** = the motor is at its limit and can't hold or follow the command. |

### 4.3 Recording a move

1. Move the arm to the start pose (sliders, preset or IK) and press
   **Set start = current pose**.
2. Move to the end pose and press **Set end = current pose**. Type a name.
3. Press **Record**. The arm goes to the start, settles for 1 s, makes the
   move, then holds for 0.5 s.

The result is saved in `recordings/<date>_<time>_<name>/`:

| File | Content |
|---|---|
| `torques.csv` | One row per frame (60 per second). For each joint: commanded angle, actual angle, speed, **motor torque**, static torque, saturated (0/1); plus the TCP position. Opens in Excel. |
| `summary.txt` | Each motor's peak torque, when it happens, the static peak, the limit, and whether it saturated. |
| `motion.mp4` | 3D view, side view and a torque plot with a moving time cursor. Rendered in the background (10–30 s). Re-render with `python make_video.py recordings/<folder>`. |
| `motion.npz`, `video_log.txt` | Data used by the video; the video's log (look here if the MP4 is missing). |

### 4.4 Design editing

Open **Design**, choose an item and move it. A **yellow box** (±0.15 m,
`DESIGN_BOX`) shows the space around the item's anchor and a **pink dot**
shows the item. When you release a slider (or press Enter), the model
rebuilds and the arm settles briefly. **Save to arm_params.py** writes the
changes into the file (the previous version is kept as `arm_params.py.bak`)
and regenerates `urc_arm.urdf`. Until you save, the status says "not saved".

| Item | What it sets | Measured from |
|---|---|---|
| shoulder pivot | Shoulder axis location (its z sets the shoulder height: 0.55 + 0.0596 = 0.6096 m) | Turntable bearing centre |
| link1: shoulder pivot | Where the shoulder axis passes through the upper-arm tube | Tube's near end |
| link1: elbow pivot | Where the elbow axis is on the upper-arm tube | Tube's far end |
| link2: elbow pivot | The same elbow axis, on the forearm tube | Tube's near end |
| link2: wrist pivot | Where the wrist axis is on the forearm tube | Tube's far end |
| motor: … / mass: … | Centre of mass of a motor or other part; mass is also editable | The anchor in its name (`pivot`, `tube_start`, `tube_end`) on its body |

Axes for tube anchors: **x along the tube** (near end → far end), **y
left**, **z perpendicular to the tube** in the plane of motion (up when the
link points forward). Examples:

* Elbow pivot `(-0.03, 0, -0.04)`: the elbow axis is 3 cm in from the end of
  the upper-arm tube and 4 cm below it.
* Shoulder pivot on link 1 `(0.05, 0, 0)`: the tube sticks out 5 cm behind
  the shoulder (a counterweight stub).
* Elbow motor at `(-0.02, -0.05, -0.03)` from `tube_end`: 2 cm back from the
  tube end, 5 cm to the right, 3 cm below.

**Moving a pivot moves the tube, not the axis.** L1 and L2 are fixed axis to
axis (they come from the mission targets), so the tube's length and tilt
change instead. `python build_urdf.py` prints them.

To change *which body* a motor is bolted to, or its anchor, edit `body` and
`anchor` in `arm_params.py` (e.g. elbow motor on the forearm:
`body="link2", anchor="tube_start"`).

### 4.5 Editing `arm_params.py`

Open it in any text editor (`gedit arm_params.py` or VS Code). Sections:

| Setting | Meaning |
|---|---|
| `L1`, `L2`, `L_GRIPPER` | Axis-to-axis lengths (m) |
| `TURNTABLE_HEIGHT`, `ROVER` | Mount and rover body geometry |
| `DENSITY`, `SECTION_MM`, `WIRING_MASS` | Tube material and rectangular cross-section (width, height, wall in mm; wall 0 = solid) |
| `SHOULDER_PIVOT`, `LINK_GEOMETRY` | Pivot offsets (4.4) |
| `JOINTS` | Per joint: min, max (deg), effort limit (N·m), max speed (deg/s) |
| `MOTORS`, `EXTRA_MASSES` | Body, anchor, position, mass (and size) of each part |
| `COM_OVERRIDE` | Force a body's centre of mass, e.g. from CAD |
| `PAYLOAD` | Box at the TCP; set `mass=5.0` for the URC box |
| `HOME_DEG`, `PRESETS` | Home pose and preset buttons |
| `DRIVE_KE`, `DRIVE_KD` | Motor position-control stiffness and damping |

Preset formats: `[x, y, z, pitch_deg]` (TCP target, solved by IK),
`dict(joints_deg=[base, shoulder, elbow, wrist, gripper_yaw])`, `"home"` or
`"straight"`. `NOSE` (0.35 m) is the rover front and `DECK` (0.50 m) the deck
height, so `[NOSE + 0.15, 0.0, 0.40, -90.0]` is 15 cm past the nose, 40 cm
up, gripper pointing down.

Restart the sim after editing the file. `python build_urdf.py` prints a
quick summary without opening the window.

---

## 5. How the sim works

### 5.1 Coordinates

* **World frame:** origin on the ground directly below the arm mount.
  **X forward** (toward the rover nose), **Y left**, **Z up**. Metres.
* **Pitch joints** (shoulder, elbow, wrist) rotate about **−Y**: positive
  angles lift the link, 0 points it forward along +X.
* **Angles are relative**: each joint's angle is measured from the previous
  link, which is what the motor turns. The absolute angle of a link is the
  sum of the joints before it.
* **Base yaw** turns the whole arm about +Z. **Gripper yaw** turns the
  gripper about its own axis.
* **TCP** (tool centre point) = gripper tip. IK targets are TCP positions.
* Torques in N·m; positive = the joint's positive direction.

### 5.2 Kinematic chain

```
base_link ─fixed─ rover ─base_yaw (Z)─ turntable ─shoulder (−Y)─ link1
   ─elbow (−Y)─ link2 ─wrist_pitch (−Y)─ wrist ─gripper_yaw (Z)─ gripper ─fixed─ tcp
```

Each moving body's frame has its origin **on its own joint axis**. For link1
and link2, x points along the **pivot line** (this axis → next axis), so the
next joint is at distance L1 / L2 along x. Joint angles are between pivot
lines, so the kinematics are exactly those of an ideal arm with lengths
L1 and L2, whatever the tube offsets are.

### 5.3 Tube geometry from pivot offsets

For a link with pivot-to-pivot length L, near offset n (own pivot from the
tube's near end) and far offset f (next pivot from the tube's far end), in
tube axes:

* tube length: T = nₓ − fₓ + √(L² − (f_z − n_z)²)
* tilt of the tube relative to the pivot line: γ = atan2(f_z − n_z, T + fₓ − nₓ)

The tube is then placed and rotated in the link frame so both pivots land
exactly L apart. Geometry that can't work (|f_z − n_z| ≥ L, or T too short)
is rejected with a message. Sideways (y) offsets shift the next link
sideways; the IK's base yaw compensates for the total offset.

### 5.4 Mass model

Each body's mass, centre of mass and inertia are built from its parts:

* **Tube:** rectangular tube, mass = density × area × T, with its exact
  inertia, rotated by the tilt γ;
* **Wiring:** `WIRING_MASS` spread over the tubes by length;
* **Motors:** solid cylinders with the axis parallel to the joint they drive;
* **Extra masses and payload:** solid boxes.

The parts are combined with the parallel-axis theorem. `COM_OVERRIDE`
replaces a body's computed COM. A small "armature" (0.01 kg·m²) is added to
each joint, standing in for motor rotor inertia.

### 5.5 Physics

* `sim_arm.py` turns the params into URDF text (`build_urdf.build`) and
  loads it into Newton; the model is never read from a stale file.
* Newton's **MuJoCo solver** integrates the dynamics at 600 steps per
  second (10 substeps per 60 Hz frame). Contacts are off.
* Each joint is a **position-controlled motor**:
  τ = K_e(θ_cmd − θ) + K_d(θ̇_cmd − θ̇), limited to ±effort. The velocity
  feed-forward (θ̇_cmd) removes lag during moves. If the needed torque
  exceeds the effort limit, the motor saturates and the arm sags or falls
  behind, as a real undersized motor would.
* Moves follow a **quintic** profile, θ(t) = θ₀ + Δθ(10u³ − 15u⁴ + 6u⁵) with
  u = t/T: zero velocity and acceleration at both ends. T is the shortest
  time that keeps every joint under its speed limit (peak speed =
  1.875 Δθ/T). The command is updated every substep.

### 5.6 Torques

* **Motor torque** (readout and recordings) = MuJoCo's actuator force,
  averaged over each frame. During a move it includes gravity, inertia
  (acceleration) and Coriolis/centrifugal effects. It was verified against
  MuJoCo's inverse dynamics of the same motion (within about 1 % on the
  shoulder and elbow).
* **Static torque** = the torque to hold the current pose against gravity:
  the moment of the weight of every mass beyond a joint about that joint's
  axis, τₖ = −aₖ · Σ (rᵢ − pₖ) × mᵢg. Same method as the MATLAB sim. At
  rest the motor torque equals it; the headless test checks this.
* Jerk isn't a separate output: torque depends on position, speed and
  acceleration, and jerk is the slope of the torque curve.

### 5.7 Inverse kinematics

Given a TCP target (x, y, z) and gripper pitch φ:

1. Base yaw points the arm plane at the target (allowing for any sideways
   pivot offset).
2. The wrist point is the TCP minus the gripper length along φ.
3. Shoulder and elbow solve the two-link triangle analytically (law of
   cosines). There are two solutions (elbow up / down).
4. The wrist angle is φ minus the shoulder and elbow angles.
5. Angles are wrapped into the joint limits; solutions outside them are
   dropped. Collisions are not checked.

### 5.8 Saving design changes

`params_io.py` rewrites only the blocks between `# >>> AUTO:<NAME>` and
`# <<< AUTO:<NAME>` in `arm_params.py` (pivots, motors, extra masses). It
checks the new file is valid Python before writing and keeps a `.bak`.
Comments inside those blocks are not preserved.

---

## 6. Working with git

```bash
git pull                         # get teammates' changes first
# ...edit arm_params.py or save from the Design panel...
python build_urdf.py             # keep urc_arm.urdf in sync
git add arm_params.py urc_arm.urdf
git commit -m "Move elbow motor below the tube end"
git push
```

* Commit `arm_params.py` and `urc_arm.urdf` together so the URDF always
  matches the params.
* `recordings/`, `.venv/` and `*.bak` are ignored
  (`.gitignore`). Share a specific recording by attaching it elsewhere, or
  force-add it with `git add -f`.
* If two people edit `arm_params.py`, git may report a conflict: open the
  file, keep the right values between the `<<<<<<<` / `>>>>>>>` markers,
  then `git add` and commit.

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ModuleNotFoundError: newton` or `matplotlib` | The venv isn't active. Run `source .venv/bin/activate` from this folder; the prompt must show `(.venv)`. |
| `error: externally-managed-environment` from pip | Same: you're in the system Python. Activate the venv. |
| `MuJoCo backend not installed` | Install with the extras: `pip install -r requirements.txt` (`newton[examples]`). |
| No `motion.mp4` in a recording | Check `video_log.txt`. Usually ffmpeg or matplotlib is missing (3.5). Re-render: `python make_video.py recordings/<folder>`. |
| "There is no app installed for MPEG-4 video" | `sudo apt install vlc`, or drag the file into Chrome. |
| Status says "out of reach" / "outside joint limits" | The IK target can't be reached with the current lengths/limits. Try another pitch or Elbow up/down. |
| Status says "rejected: …" after a design change | That geometry is impossible (e.g. pivot offsets bigger than the link); the change was undone. |
| Arm droops and the readout shows SAT | That motor's effort limit is too low for the pose. Raise the limit in `JOINTS` or change the design. |
| The arm dips after a design change | Normal: the model was rebuilt and is settling. |
| `python -m newton.examples` window doesn't open / very slow | Check the NVIDIA driver (`nvidia-smi`, 3.2). |

---

## 8. Limits and placeholders

* **No collisions:** the rover and ground are drawn, but the arm passes
  through them, and the IK doesn't check for collisions.
* **Rigid links and ideal motors:** no bending, backlash, belt stretch or
  motor electrical limits, so vibration from jerky moves isn't shown.
* **Placeholders to replace when hardware is chosen** (marked `PLACEHOLDER`
  in `arm_params.py`): motor masses and sizes, effort limits, joint speeds,
  turntable height, gripper actuator and camera masses.
* Joint limits default to ±180° (gripper yaw ±90°).
* Payload is off by default (`PAYLOAD["mass"] = 0`).

Ideas for later: collisions with the rover and ground, joint compliance
(gearbox stiffness), jerk and torque-rate columns in the CSV, a 7th-order
move profile with zero jerk at the ends, and a ROS 2 bridge for the real
encoders.
