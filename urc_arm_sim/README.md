# URC Rover Arm Simulation (Newton + URDF)

A physics simulation of our rover arm, used to check reach, poses and **motor
torques** before hardware is built. It runs in
[NVIDIA Newton](https://github.com/newton-physics/newton) (a GPU physics
engine built on MuJoCo) and generates a **URDF** robot description of the
arm.

The arm: a **sliding base** that moves the whole arm left/right across the
rover (a prismatic joint along Y, ±0.35 m), **shoulder**, **elbow** and
**wrist** (pitch), and a **wrist roll** (about the gripper's own axis). Link 1 = 1.0 m and link 2 = 0.84 m
axis to axis, gripper 0.15 m. A turntable (yaw) base is available instead
with one setting (`BASE_JOINT`, 4.6). It is the successor to the team's MATLAB arm sim and
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
| `arm_params_4bar.py` | The **4-bar link 1** design: same as `arm_params.py` but link 1 is four braced corner tubes (4.9). |
| `arm_params_2bar.py` | The **2-bar link 1** design: link 1 is two braced tubes, one above the other, in the arm's plane (4.9). |
| `arm_params.py` | **The file you edit.** Lengths, tube section and material, pivot offsets, joint limits / effort / speed, motors, extra masses, COM overrides, payload, home pose, preset targets. |
| `sim_arm.py` | The simulation and its side panel (sliders, presets, IK, collision checking, recording, design editing). |
| `arm_model.py` | The arm's math: geometry, mass properties, forward/inverse kinematics, static torques, collision checks. |
| `build_urdf.py` | Writes `urc_arm.urdf` from the params and prints masses, tube lengths and holding torques. |
| `make_video.py` | Turns a recording into an MP4 (the sim runs it automatically). |
| `sections.py` | Link cross-sections (rectangular tube, laced or battened corner bars): stiffness, mass, stress. |
| `deflection.py` | Link bending, torsion and stress under the live loads (4.9). |
| `compare_designs.py` | Prints the designs side by side (tube, 4-bar, 2-bar by default): masses, stiffness, deflection, safety factors, torques, tipping. |
| `scene.py` | Task objects: the box with its handle, the drawer cabinet and sliding tray. |
| `mission.py` | The box → drawer mission: plan, check, rover-pose search, reach map. |
| `params_io.py` | Writes design and task changes made in the sim back into `arm_params.py`. |
| `urc_arm_4bar.urdf`, `urc_arm_2bar.urdf` | Generated URDFs of the 4-bar and 2-bar designs (`python build_urdf.py arm_params_4bar`). |
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
git clone https://github.com/CU-URC-2027/ROS2-URDF-Newton-Sim.git
cd ROS2-URDF-Newton-Sim/urc_arm_sim          # adjust to where this folder sits in the repo
```
May be better to just download this repo entirely, in the next sections, simply replace the CU-URC-2027/ROS2-URDF-Newton-Sim directories with wherever you installed and unzipped this repo
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

The arm stands on a grey box (the rover body). The dark bar across the deck
is the slide rail, with the mount post riding on it. Blue boxes are the link
tubes, orange cylinders are motors, grey boxes are other masses, red rods
mark the pitch axes and the green ball is the TCP (gripper tip). Parts that
are colliding are traced in **red** (4.3).

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
| **Check collisions** | Turns collision checking on/off (4.3). On by default. |
| **Joints** | `base_slide` in **metres**; the other joints in **relative degrees** (what each motor turns). The arm moves there smoothly, limited by each joint's speed. |
| **Presets** | Buttons for saved poses. Edit `PRESETS` in `arm_params.py`; their TCP coordinates are **relative to the rover** (its frame, wherever it is). Below them: **Carrying box** puts the 5 kg task box in the gripper (its mass, the drawn box and its collisions), and a drop-down that also holds the **mission poses** (`M …`) after a **Check mission** (4.4). |
| **TCP target / IK** | Type a gripper-tip position (m) and pitch (deg; −90 = pointing down, 0 = level) and press **Solve IK**. The slide moves the arm sideways to line up with the target's Y (a turntable base turns to face it instead). **Elbow up** picks between the two arm solutions; if one collides, the other is used. |
| **Record move** | Records a move from a start pose to an end pose (4.4). |
| **Platform and stability** | Chassis size, deck height, mass and COM; arm mount position; wheels; slope (4.8). |
| **Task: box → drawer** | Place the box, set grasps and rover poses, check or run the mission, make a reach map (4.7). |
| **Design** | Move pivots, motors and masses live and save them (4.5). |
| **Readout** | TCP position; **tip margin** (now, and if braking/turning), **wheel loads**, **arm mount loads**; **collision status** (green "clear" or red "COLLISION: …"); and for every motor: position, torque now (slide: force in N), static (holding) torque, and effort limit. **SAT** = the motor is at its limit and can't hold or follow the command. |

### 4.3 Collision checking

The sim checks every frame whether any part of the arm touches the **ground**,
the **rover body**, the **mount post**, a motor on the rover, or **another
part of the arm**. It warns rather than blocking: the arm still moves, but

* the readout shows what is hitting what, e.g. `COLLISION: link2 tube hits rover body`;
* the colliding parts are traced in red in the 3D view;
* the IK and presets skip solutions that collide (and say so);
* recordings log collisions per frame (4.4).

The checks use simple shapes: tubes and the gripper are capsules (radius =
half the larger tube side), motors and extra masses are spheres, the payload
is a box, the rover is a box and the post is a capsule. Parts on neighbouring
bodies meet at their joint by design, so they aren't checked against each
other, except two tubes folding onto each other (the region right at the
joint is ignored). `COLLISION_MARGIN` in `arm_params.py` adds extra
clearance (e.g. `0.01` = 1 cm). Untick **Check collisions** to turn it all off.

### 4.4 Recording a move

1. Move the arm to the start pose (sliders, preset or IK) and press
   **Set start = current pose**.
2. Move to the end pose and press **Set end = current pose**. Type a name.
3. Press **Record**. The arm goes to the start, settles for 1 s, makes the
   move, then holds for 0.5 s.

Or pick poses from the preset drop-down and press **Start = preset** /
**End = preset**. To record with the box, tick **Carrying box** first.

**Mission poses.** Each **Check mission** that plans successfully adds the
pose at the end of every arm move of the mission (e.g.
`M pick_up: lift box`, `M place: move over the tray`) to the drop-down. Each one
stores the TCP relative to the rover, the gripper pitch and wrist roll, and the
scene at that moment: whether the box is in the gripper and how far the drawer
is open. **Go** or **Start = preset** applies that scene too, so a pose taken
while the box is held turns **Carrying box** on. The rover doesn't move; the
drawer and cabinet stay where they are in the world, so collisions with them
are only like the mission's if the rover is at that phase's pose (Task →
**drive here**). The recorded move is a straight joint move from start to end,
not the mission's path, which is the point: it shows what that shortcut would
do (e.g. `lift box` → `move over the tray` sweeps the box through the open
drawer; the mission raises the box first). Mission poses are kept until the
next check; they aren't saved to `arm_params.py`.

The result is saved in `recordings/<date>_<time>_<name>/`:

| File | Content |
|---|---|
| `torques.csv` | One row per frame (60 per second). For each joint: commanded position, actual position, speed, **motor torque** (slide: force), static torque, saturated (0/1); the TCP position; tipping and wheel/mount loads; **deflection** (TCP total and downward, each link's tip deflection, twist and least safety factor); `box_held`; and a `collision` column (empty = clear). Column names carry the units (`_deg`, `_m`, `_Nm`, `_N`, `_mm`). Opens in Excel. |
| `summary.txt` | Each motor's peak torque, when it happens, the static peak, the limit, whether it saturated; whether the box was carried; least tip margin, lightest wheel and mount loads; peak deflection and least safety factor per link; and the time spans with collisions. |
| `motion.mp4` | 3D view, side view and a torque plot with a moving time cursor. Colliding parts turn red, a red banner names them, and collision times are shaded on the plot. Rendered in the background (10–30 s). Re-render with `python make_video.py recordings/<folder>`. |
| `motion.npz`, `video_log.txt` | Data used by the video; the video's log (look here if the MP4 is missing). |

### 4.5 Design editing

Open **Design**, choose an item and move it. A **yellow box** (±0.15 m,
`DESIGN_BOX`) shows the space around the item's anchor and a **pink dot**
shows the item. When you release a slider (or press Enter), the model
rebuilds and the arm settles briefly. **Save to arm_params.py** writes the
changes into the file (the previous version is kept as `arm_params.py.bak`)
and regenerates `urc_arm.urdf`. Until you save, the status says "not saved".

| Item | What it sets | Measured from |
|---|---|---|
| shoulder pivot | Shoulder axis location (its z sets the shoulder height: 0.55 + 0.0596 = 0.6096 m) | Base centre (top of the carriage) |
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

### 4.6 Editing `arm_params.py`

Open it in any text editor (`gedit arm_params.py` or VS Code). Sections:

| Setting | Meaning |
|---|---|
| `L1`, `L2`, `L_GRIPPER` | Axis-to-axis lengths (m) |
| `BASE_JOINT` | `"slide_y"` (prismatic, along Y; default) or `"yaw"` (turntable about Z) |
| `CHASSIS`, `ARM_MOUNT`, `WHEELS`, `SLOPE` | The platform (4.8): chassis box, mass and COM; where the arm is mounted on the deck; wheel layout; ground slope |
| `BASE_ABOVE_DECK` | Top of the carriage / turntable above the deck (base height = deck + this) |
| `DRIVE_ACCEL` | Braking / accelerating / turning acceleration for the worst-case tipping check |
| `LINK_SECTIONS` | Other section types per link, overriding `SECTION_MM` (4.9) |
| `MATERIAL` | E, G and yield strength for stiffness and stress |
| `DESIGN_NAME`, `DEFLECTION_SCALE` | Label shown in the readout / recordings; how much the drawn bent shape is exaggerated |
| `DENSITY`, `SECTION_MM`, `WIRING_MASS` | Tube material and rectangular cross-section (width, height, wall in mm; wall 0 = solid) |
| `SHOULDER_PIVOT`, `LINK_GEOMETRY` | Pivot offsets (4.5). Default: link2 runs 4 cm to the left of link1 (side by side) |
| `JOINTS` | Per joint: min, max (deg), effort limit (N·m), max speed (deg/s). `base_slide` uses metres, N and m/s; `base_yaw` is used only with the turntable |
| `MOTORS`, `EXTRA_MASSES` | Body, anchor, position, mass (and size) of each part |
| `COM_OVERRIDE` | Force a body's centre of mass, e.g. from CAD |
| `PAYLOAD` | Box at the TCP; set `mass=5.0` for the URC box |
| `HOME_DEG`, `PRESETS` | Home pose and preset buttons (`base` value in metres for the slide) |
| `DRIVE_KE`, `DRIVE_KD` | Motor position-control stiffness and damping |
| `COLLISION_MARGIN` | Extra clearance for the collision check (m) |

Preset formats: `[x, y, z, pitch_deg]` (TCP target, solved by IK),
`dict(joints_deg=[base, shoulder, elbow, wrist, wrist_roll])` (base in
metres for the slide), `"home"` or
`"straight"`. `NOSE` (0.35 m) is the rover front and `DECK` (0.50 m) the deck
height, so `[NOSE + 0.15, 0.0, 0.40, -90.0]` is 15 cm past the nose, 40 cm
up, gripper pointing down.

Restart the sim after editing the file. `python build_urdf.py` prints a
quick summary without opening the window.

### 4.7 Task: box → drawer

The task scene has the **box** (40 cm envelope including the handle, 5 kg,
handle bar on top), a **drawer cabinet** with a tray that pulls out, a blue
**box zone** on the ground, and the **rover**, which can now drive. The
mission:

1. **Open the drawer:** approach the drawer handle, grip, pull the tray out
   just far enough for the box ("exact fit"), release, back off.
2. **Pick up the box:** stow, drive if needed, approach above the handle,
   lower, grip (the box's mass is added to the gripper from here on), lift.
3. **Put it in the drawer:** raise the box above the tray rim, drive if
   needed, move over the tray, lower, release, back off, move clear of the
   drawer, stow.

Straight-line moves (reaching handles, pulling, lifting, lowering) keep the
gripper pitch fixed and move the TCP in a straight line. Other moves are
smooth joint moves. The rover drives between phases with the arm holding
still.

**Panel controls:**

| Control | What it does |
|---|---|
| Show task scene | Shows the objects and includes them in the collision checks. |
| box x / y / yaw | Where the box sits (inside the zone; yaw turns the box and its handle). **Random spot in zone** picks one. |
| auto wrist roll | On (default): the mission picks the wrist roll that closes the jaws squarely on each handle, and re-rolls while placing so the (square) box goes into the tray square. Off: uses the roll values below. |
| box / drawer pitch (roll) | The gripper pitch for each grasp (deg; −90 = pointing down, 0 = level), and the wrist roll when auto roll is off. |
| Rover pose per phase | Where the rover stands for each phase (x, y m; yaw deg). **drive here** drives it there now. |
| **Check mission** | Plans the mission and steps through it without physics (a few seconds). Lists every step with a collision, a motor over its limit, or a grasp problem. |
| **Find rover poses** | For each phase, searches for the nearest rover pose (forward/back first, then sideways, then turning) where that phase is clear and within the motor limits, and fills in the poses. |
| **Reach map** | For every box position in the zone (5 cm grid), checks whether the arm can pick the box up from the pick-up rover pose. Draws coloured dots on the ground and saves `reach_map.png` + `.csv` in `recordings/`. |
| **Run mission** | Runs the whole mission with physics; with **record** ticked it saves a recording (4.4) with extra columns: `rover_x/y/yaw`, `drawer_open_m`, `box_held`, `step`, and a per-step torque table in `summary.txt`. The video shows the box, drawer and rover. |
| Reset scene | Box back on the ground, drawer closed, rover and arm home. |
| Save task settings | Writes box pose, grasps and rover poses into `arm_params.py`. |

**Grasp checks.** At each grip the check reports if the jaws are more than
`jaw_tol_deg` (10°) from closing squarely on the handle, or if the handle is
wider than the jaw opening (`GRIPPER["max_opening"]`). The jaws close along
the gripper's `GRIPPER["jaw_axis"]` (default `y`, parallel to the pitch axes).
These are warnings; the mission still runs. With auto wrist roll on, the
jaws are always squared to the handle, so this only matters with the roll
set by hand.

**Settings in `arm_params.py`** (task section): `TASK_BOX` (size, mass,
handle), `BOX_ZONE`, `DRAWER` (position, floor height, pull direction,
clearances, handle), `MISSION` (approach and retract distances, lift and rim
clearances, speeds, jaw tolerance, reach-map step), `GRIPPER` (jaw axis and
opening), `ROVER_KE/KD`, and the saved `BOX_POSE`, `BOX_GRASP`,
`DRAWER_GRASP`, `ROVER_POSES`.

**What it doesn't model (yet):** the box is attached to the gripper when
gripped (no jaw contact, friction or slip); the drawer follows the gripper
along its slide with no friction or pull force on the arm; the rover drives
on flat ground.

### 4.8 Platform and stability (tipping, wheel loads, mount loads)

The **platform** is the chassis top (deck). In the panel (or `arm_params.py`)
you set:

| Setting | Meaning |
|---|---|
| chassis length / width, deck height, ground clearance | The chassis box. Raising the deck raises the arm with it. |
| chassis mass, COM x / y / height | The rover without the arm; COM x, y from the chassis centre. |
| arm mount x / y | Where the arm sits on the deck, from the chassis centre. |
| wheelbase, track, wheel diameter | Wheel contact points at (±wheelbase/2, ±track/2) from the chassis centre. |
| rocker pivot x | Where the side rockers pivot, from the chassis centre. |
| slope, downhill direction | Ground slope (deg) and which way is downhill (deg from +X). |
| braking/turning check | `DRIVE_ACCEL`, the extra acceleration for the worst-case check. |

Changes rebuild the model when you release a slider. **Save platform to
arm_params.py** keeps them. The world origin stays on the ground under the
arm mount, so moving the mount moves the chassis and wheels around it.

**What you see:**

* In the 3D view, the **wheel footprint** on the ground and the **centre of
  pressure** (dot): green when the margin is over 5 cm, orange under 5 cm,
  red when tipping.
* In the readout:
  * **Tip margin:** distance from the centre of pressure to the nearest
    footprint edge (front, rear, left, right). Negative = tipping over that
    edge. The second number is the worst case if the rover also brakes,
    accelerates or turns at `DRIVE_ACCEL` in any direction.
  * **Wheel loads** (N) for FL, FR, RL, RR. **LIFT** = a wheel is lifting off.
  * **Arm mount loads:** vertical force, horizontal shear, bending moment
    and twist at the mount (deck height), for sizing the mount plate,
    bolts and bearing.
* **Recordings** add `tip_margin_m`, `tip_edge`, `tip_margin_driving_m`, the
  four wheel loads and the four mount loads to `torques.csv`, a tipping
  section to `summary.txt` (least margin and when, lightest wheel, peak
  mount bending), and the footprint and centre of pressure to the video.
* **Check mission** reports the least tip margin over the mission (and
  while driving), and flags any step where the rover tips or a wheel lifts.
  **Find rover poses** rejects poses where the rover would tip.
* `python build_urdf.py` prints the margins for the test poses.

### 4.9 Link 1 designs, and deflection

There are three design files; everything else is shared:

| Design | File | URDF | Link 1 |
|---|---|---|---|
| Tube | `arm_params.py` | `urc_arm.urdf` | 25.4 × 25.4 × 3 mm aluminium tube |
| 4-bar | `arm_params_4bar.py` | `urc_arm_4bar.urdf` | Four 10 × 10 × 1.5 mm square corner tubes on an 80 × 80 mm square, 10 × 3 mm flat-bar diagonal lacing every 150 mm on all four faces (placeholders) |
| 2-bar | `arm_params_2bar.py` | `urc_arm_2bar.urdf` | Two 10 × 10 × 1.5 mm square tubes in the arm's plane, one 80 mm above the other, joined by 10 × 3 mm flat-bar diagonal lacing every 150 mm (placeholders) |

```bash
python sim_arm.py --params arm_params_4bar      # run the 4-bar design
python build_urdf.py arm_params_4bar            # its URDF + summary
python sim_arm.py --params arm_params_2bar      # run the 2-bar design
python compare_designs.py                       # all three side by side
```

Saving from the sim writes back to whichever file you ran. Recordings of
the 4-bar and 2-bar designs get `_4bar` / `_2bar` in their folder name. In the 4-bar file, link 2
sits 6.5 cm to the side (instead of 4 cm) to clear the wider link 1, and
link 1's bars start 6 cm out from the shoulder axis (the shoulder fitting
fills the gap, clear of the mount post). The 2-bar file does the same at
the shoulder, with link 2 back at 4 cm (link 1 is only 10 mm wide).

The 4-bar section is set in `LINK_SECTIONS` (mm):

```python
LINK_SECTIONS = {"link1": dict(type="corner_bars", width=80, height=80, bar_w=10, bar_h=10, bar_wall=1.5,
                               lace_w=10, lace_h=3, lace_wall=0, bay=150)}
```

`width` / `height` are bar centre spacings. The bars and the lacing are
rectangular: `bar_w` across the arm × `bar_h` in the plane of motion,
`lace_w` wide in the face × `lace_h` deep (out of the face); `wall` 0 = solid
(so the lacing above is 10 × 3 mm flat bar). `bay` = lacing pitch. Any link
can use it.

**Battened** instead of laced: the same four bars, tied together by flat
plates welded across all four faces at every station (one per `spacing`, plus
both ends), so every bar is joined to its neighbours and each station is a
closed rectangular frame. A commented example is in `arm_params_4bar.py`:

```python
"link1": dict(type="battened", width=80, height=80, bar_w=10, bar_h=10, bar_wall=1.5,
              batten_w=20, batten_t=3, spacing=150)   # plate width along the link, thickness, pitch (mm)
```

Bending stiffness is the same as laced (the bars still act together), but
shear and torsion are carried by the bars and plates bending between
stations (frame action), which is much softer. With the placeholder sizes,
straight out with the box: link 1 tip 3.8 mm vs 2.7 mm laced, link 1 twist
0.78° vs 0.09°, and torsion stiffness GJ 319 N·m² (laced 2684, the single
tube 877). Closer plates help; with wide spacing the bars buckle between
plates.

**Two bars in the arm's plane** (`arm_params_2bar.py`): two rectangular bars,
one above the other, braced in that plane only:

```python
"link1": dict(type="two_bars", height=80, bar_w=10, bar_h=10, bar_wall=1.5,
              bracing="laced", lace_w=10, lace_h=3, lace_wall=0, bay=150)
# or bracing="battened", batten_w=20, batten_t=3, spacing=150
```

It is deep in the plane of motion, so in-plane bending is stiff (EI 11 348
N·m², half the 4-bar's, 7× the tube's) at half the tube's mass. But
sideways it is just two 10 mm bars (EI 87 N·m², 1/18 of the tube's), and it
is an open section, so torsion is very soft (GJ 48 N·m², 1/18 of the tube's)
and it can **buckle sideways** under in-plane bending (lateral-torsional
buckling, checked with the classic $M_{cr}$ formula over the link length).
Link 2 hangs 4 cm to the side, so it twists link 1 whenever the arm isn't
straight. With the placeholder sizes: straight out with the box, TCP 29.6 mm,
link 1 twist 3.1°, least safety factor 1.4 (lateral-torsional buckling); over
the mission, peak TCP deflection 31.8 mm vs 12.7 mm (4-bar) and 68.8 mm
(tube).

**Deflection** (both designs) is shown:

* in the readout: TCP deflection (and how much is downward), each link's tip
  deflection and twist, and the **least safety factor** with what limits it
  (tube yield; or for the 4-bar: corner-bar yield, corner-bar buckling
  between lacing points, lacing yield or buckling; for battened: corner-bar
  yield or buckling between battens, batten plate yield; for 2-bar: bar
yield, bar buckling between bracing points, lacing or plate yield,
lateral-torsional buckling). Tick the checkbox to
  draw the **bent shape** (magenta), exaggerated `DEFLECTION_SCALE` times;
* in recordings: `tcp_deflection_mm`, `tcp_deflection_down_mm`, each link's
  tip deflection, link 1 and 2 twist, and each link's safety factor in the
  CSV; a deflection section in `summary.txt`; the bent shape (dashed) in
  the video;
* in **Check mission** (peak TCP deflection, least safety factor; any
  overstressed step is flagged) and in `build_urdf.py` (test poses, empty
  and with the 5 kg box at the TCP).

The sim's physics still uses rigid links, so the arm, the collision checks
and the IK don't move by the deflection. The numbers say how far the real
arm would sag; with the tube design and the box at full reach that's
about 7 cm at the TCP, which matters for placing the box (`place_gap` is
only 5 mm).

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
* **Base slide** moves the whole arm along **+Y** (left), parallel to the
  pitch axes, so the arm's plane of motion shifts sideways. (With
  `BASE_JOINT = "yaw"`, the base turns the arm about +Z instead.)
* **The wrist has two joints:** **wrist pitch**, which moves the gripper in
  the arm's plane like the shoulder and elbow, then **wrist roll**, which
  turns the gripper about its own axis (the TCP doesn't move).
* **TCP** (tool centre point) = gripper tip. IK targets are TCP positions.
* Torques in N·m (slide: force in N); positive = the joint's positive direction.

### 5.2 Kinematic chain

```
base_link ─fixed─ rover ─base_slide (along Y)─ carriage ─shoulder (−Y)─ link1
   ─elbow (−Y)─ link2 ─wrist_pitch (−Y)─ wrist ─wrist_roll (X, along the gripper)─ gripper ─fixed─ tcp
```

Each moving body's frame has its origin **on its own joint axis**. For link1
and link2, x points along the **pivot line** (this axis → next axis), so the
next joint is at distance L1 / L2 along x. Joint angles are between pivot
lines, so the kinematics are exactly those of an ideal arm with lengths
L1 and L2, whatever the tube offsets are. The carriage's frame sits at
(0, slide, `BASE_HEIGHT`); the mount post is part of it. With the yaw option
the base link is called `turntable` and the joint `base_yaw` (about Z).

### 5.3 Tube geometry from pivot offsets

For a link with pivot-to-pivot length L, near offset n (own pivot from the
tube's near end) and far offset f (next pivot from the tube's far end), in
tube axes:

* tube length: T = nₓ − fₓ + √(L² − (f_z − n_z)²)
* tilt of the tube relative to the pivot line: γ = atan2(f_z − n_z, T + fₓ − nₓ)

The tube is then placed and rotated in the link frame so both pivots land
exactly L apart. Geometry that can't work (|f_z − n_z| ≥ L, or T too short)
is rejected with a message. Sideways (y) offsets shift the next link
sideways; the IK compensates for the total offset (4 cm by default).

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
  τ = K_e(θ_cmd − θ) + K_d(θ̇_cmd − θ̇) + τ_g, limited to ±effort (for the
  slide the same law gives a force in N from position in m). τ_g is
  **gravity feed-forward** (`GRAVITY_FF`): the static torque the commanded
  pose needs, as a real controller with gravity compensation would add.
  Without it a position controller sags under load (about 1° at the
  shoulder for 100 N·m), which is enough to push a held box into the drawer.
* The **rover** is driven by three position-controlled joints (x, y, yaw)
  between the ground and the rover body (`ROVER_KE/KD`), so its
  accelerations reach the arm. The velocity
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
  rest the motor torque equals it; the headless test checks this. On a level
  rail the slide's static force is zero: it only needs force to accelerate
  the arm (mass × acceleration), plus whatever friction the real rail has.
* Jerk isn't a separate output: torque depends on position, speed and
  acceleration, and jerk is the slope of the torque curve.

### 5.7 Inverse kinematics

Given a TCP target (x, y, z) and gripper pitch φ:

1. The slide moves the arm plane to the target's Y (slide = y − sideways
   offset); if that's outside the slide's travel, the status says how far it
   would need to go. (Turntable option: base yaw points the arm plane at the
   target instead.)
2. The wrist point is the TCP minus the gripper length along φ.
3. Shoulder and elbow solve the two-link triangle analytically (law of
   cosines). There are two solutions (elbow up / down).
4. The wrist angle is φ minus the shoulder and elbow angles.
   The wrist roll doesn't move the TCP, so it is simply set to the
   requested value (in the mission: the auto roll that squares the jaws to
   the handle, θ = atan2(−a·b, c·b) with a and c the jaw directions at roll
   0 and 90° and b the handle bar).
5. Angles are wrapped into the joint limits; solutions outside them are
   dropped. With collision checking on, colliding solutions are dropped too.

### 5.8 Collision checks

For each pose the shapes in 4.3 are placed with forward kinematics and
tested: capsule–capsule by closest distance between segments, sphere tests by
point distance, capsule vs the rover box by a slab test on the box grown by
the radius, payload box vs rover box by the separating-axis test, and
anything vs the ground by its lowest point. A pair hits when the gap is
below the sum of radii plus `COLLISION_MARGIN`. Only poses are checked, once
per frame; a very fast move could pass through something between frames.

### 5.9 Tipping and wheel loads

Everything on the rover is a point mass: the chassis (at its COM), the
rover-mounted parts, and each arm body (with a held box added to the
gripper). In the sim, each mass's acceleration comes from its position over
the last three frames (lightly smoothed), so arm swings and rover driving
both count. The ground must supply

$$\mathbf F=\sum m_i(\mathbf a_i-\mathbf g),\qquad \mathbf M_O=\sum \mathbf r_i\times m_i(\mathbf a_i-\mathbf g)$$

(rotational inertia of each body is neglected). The **centre of pressure**
is where this load meets the ground: $x_c=-M_y/F_z$, $y_c=M_x/F_z$ in the
rover frame. The **tip margin** is its distance to the nearest edge of the
wheel rectangle.

**Wheel loads (rocker-differential).** Four unknown normal forces $N_i$, four
equations: vertical force, the two tipping moments, and the differential
condition. The differential makes the two rockers turn equal and opposite,
so (by virtual work) they carry equal moments about their pivots:

$$N_{FL}(x_F-x_p)+N_{RL}(x_R-x_p)=N_{FR}(x_F-x_p)+N_{RR}(x_R-x_p)$$

A negative $N_i$ means that wheel lifts off. Friction forces lie in the
ground plane and don't enter these equations.

**Mount loads:** the same sums over the arm bodies only, about the mount
point on the deck, expressed in the rover's axes.

**Slope:** gravity is tilted in the physics and in every calculation
(static torques, gravity feed-forward, stability); the drawn ground stays
level. **Driving check:** the stability is recomputed with an extra uniform
acceleration of `DRIVE_ACCEL` forward, backward, left and right, and the
worst case is reported.

### 5.10 Link sections and deflection

**Sections** (`sections.py`), link frame: x along the link, y across the
arm, z in the plane of motion.

* Rectangular tube: the usual $I_y=(bh^3-b_ih_i^3)/12$ (in-plane),
  $I_z=(hb^3-h_ib_i^3)/12$, Bredt torsion $J=4A_m^2t/p_m$, webs carrying
  shear.
* Braced corner bars (rectangular bars of area $A_b$, inertias $I_{b,y}$, $I_{b,z}$ at the corners of
  a $w\times h$ rectangle; lacing of area $A_d$ at angle $\varphi$ to the
  link axis, one diagonal per bay on each face):
  * bending: $EI_y=E(4I_{b,y}+A_b h^2)$, $EI_z=E(4I_{b,z}+A_b w^2)$;
  * shear stiffness of a laced face (Timoshenko, built-up members):
    $S=EA_d\cos\varphi\sin^2\varphi$; two faces carry each shear direction;
  * torsion: the four laced faces form a closed cell,
    $GJ=4(wh)^2/\sum_i(b_i^2/S_i)$.
  * stress: a corner bar carries $|N|/4+|M_y|/(2h)+|M_z|/(2w)$, checked for
    yield and Euler buckling over one bay; a diagonal carries the face
    shear (bending shear + torsion shear flow) times $d/b$, checked for yield
    and buckling over its length.
* Battened corner bars (plates of width $b_p$, thickness $t_p$ every $a$,
  $I_p=t_pb_p^3/12$; EN 1993-1-1 §6.4.3):
  * bending as laced;
  * shear stiffness of a face with two chords $h_0$ apart:
    $S_v=\dfrac{24EI_{ch}}{a^2\,(1+2I_{ch}h_0/(I_pa))}\le\dfrac{2\pi^2EI_{ch}}{a^2}$, with $I_{ch}$ the bar's $I$ for bending in that face;
    torsion from the same closed-cell formula with these $S_v$;
  * stress: a face's shear $V_f$ bends each bar by $V_fa/4$ between plates
    (on top of its axial force; yield, and buckling over $a$ with that
    bending) and each plate by $V_fa/2$ (with shear $V_fa/h_0$); welds are
    not checked.
* Two bars in the arm's plane, $h$ apart: $EI_y=E(2I_{b,y}+A_bh^2/2)$,
  $EI_z=2EI_{b,z}$, $GJ=2GJ_b$ (open section), warping constant
  $I_w=I_{b,z}h^2/2$; one braced face carries the in-plane shear (as above).
  Lateral-torsional buckling:
  $M_{cr}=\frac{\pi}{L}\sqrt{EI_zGJ}\sqrt{1+\frac{\pi^2EI_w}{GJL^2}}$ with
  $L$ the link length (uniform moment; conservative for the arm's moment,
  which is largest at the root).

**Deflection** (`deflection.py`): link 1, link 2 and the gripper are beams
along their pivot lines, each clamped at its proximal joint (joints and
motors rigid). Every mass is a point load $m(\mathbf g-\mathbf a)$ (tubes
split into pieces, a held box included, accelerations from the motion). At
each station the internal force and moment come from all loads beyond it;
integrating $\theta'=(M_x/GJ,M_y/EI_y,M_z/EI_z)$ and
$u'=\theta\times\hat x+(0,V_y/S_y,V_z/S_z)$ gives each link's tip
deflection and rotation, which carry the later links. Checked against
hand calculations (tip load on a straight arm, torsion from the side
offset, root stress): within 0.01 %.

Not included: joint play and gearbox wind-up, the bars' end fittings, local
buckling of the tube walls.

### 5.11 Saving design changes

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
| Status says "needs the slide at … m" | The target's Y is beyond the slide's travel. Move the target or widen `JOINTS["base_slide"]`. |
| Status says "every solution collides: …" | Both elbow choices hit something. Change the target or pitch, or untick Check collisions to go there anyway. |
| Readout shows red COLLISION | The current pose touches something; the message says what. |
| Status says "rejected: …" after a design change | That geometry is impossible (e.g. pivot offsets bigger than the link); the change was undone. |
| Arm droops and the readout shows SAT | That motor's effort limit is too low for the pose. Raise the limit in `JOINTS` or change the design. |
| The arm dips after a design change | Normal: the model was rebuilt and is settling. |
| `python -m newton.examples` window doesn't open / very slow | Check the NVIDIA driver (`nvidia-smi`, 3.2). |

---

## 8. Limits and placeholders

* **Deflection is calculated, not simulated:** the physics uses rigid
  links; joints are rigid in the deflection model (4.9).
* **Tipping is calculated, not simulated:** the rover never actually tips
  in the sim (its pose is held by the drive joints); the margin and wheel
  loads say when it would. Each body's rotational inertia is left out of the
  tipping calculation; wheels are points on flat ground; the chassis and
  rockers are rigid.
* **Task simplifications** (4.7): grasping attaches the box (no contact or
  slip), the drawer follows the gripper without resistance, flat ground.
* **Collisions are checked, not simulated:** the sim warns and the IK avoids
  colliding poses, but the arm still passes through the rover and ground
  physically (no contact forces). Checks use simplified shapes and run once
  per frame.
* **Rigid links and ideal motors:** no bending, backlash, belt stretch or
  motor electrical limits, so vibration from jerky moves isn't shown.
* **Placeholders to replace when hardware is chosen** (marked `PLACEHOLDER`
  in `arm_params.py`): motor masses and sizes, effort limits, joint speeds,
  base height, slide travel / force / speed, carriage mass, gripper actuator
  and camera masses, chassis mass and COM, wheelbase, track, wheel size,
  rocker pivot, `DRIVE_ACCEL`, the 4-bar section sizes.
* Joint limits default to ±180°; the slide to ±0.35 m.
* The slide is assumed level: rover tilt (driving on a slope) isn't modelled,
  which would add a gravity load on the slide motor.
* Payload is off by default (`PAYLOAD["mass"] = 0`).

Ideas for later: physical contacts (e.g. with a drawer), rover tilt, joint compliance
(gearbox stiffness), jerk and torque-rate columns in the CSV, a 7th-order
move profile with zero jerk at the ends, and a ROS 2 bridge for the real
encoders.
