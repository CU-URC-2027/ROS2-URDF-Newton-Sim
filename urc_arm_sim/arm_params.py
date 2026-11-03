"""
URC rover arm parameters (3-DOF pitch arm on a sliding base).

Edit this file, then run:
    python build_urdf.py      # writes urc_arm.urdf and prints masses / torques
    python sim_arm.py         # opens the Newton sim

The blocks between  # >>> AUTO  and  # <<< AUTO  are rewritten when you press
"Save to arm_params.py" in the sim (a backup goes to arm_params.py.bak).
Edit them by hand too, but keep comments outside those blocks.

Conventions match the MATLAB sim (arm_simulation_README.md, section 2):
  * World origin = arm mount point projected to the ground.
  * X forward (rover nose), Y left, Z up. Metres, kg, degrees in this file.
  * Pitch joints rotate about -Y: positive lifts the link toward +Z,
    0 = link pointing along +X. Angles are RELATIVE (what each motor turns)
    and are measured between PIVOT LINES (pivot-to-pivot), not tubes.
  * The wrist has two joints: wrist_pitch (in the arm's plane, like the
    shoulder and elbow) and wrist_roll (about the gripper's own axis).
  * The base joint (BASE_JOINT) moves the whole arm:
      "slide_y" - prismatic: slides along Y (left/right across the rover),
                  parallel to the pitch axes. Default.
      "yaw"     - turntable: rotates about +Z (vertical).

Values marked PLACEHOLDER are guesses; replace them when hardware is chosen.
"""

# ---------------------------------------------------------------- geometry
# Pivot-to-pivot distances (axis to axis, in the plane of motion). These set
# reach and the IK; the tube lengths follow from the pivot offsets below.
L1 = 1.0             # shoulder axis -> elbow axis (m)  [team decision, Nov 2026]
L2 = 0.84            # elbow axis -> wrist axis (m)     [team decision, Nov 2026]
L_GRIPPER = 0.15     # wrist axis -> TCP (m)

BASE_JOINT = "slide_y"     # "slide_y" (prismatic, along Y) or "yaw" (turntable about Z)
BASE_ABOVE_DECK = 0.05     # top of the carriage / turntable bearing above the deck (m). PLACEHOLDER
# Base height = deck height + BASE_ABOVE_DECK (0.55 m); shoulder height = base height +
# SHOULDER_PIVOT z (0.6096 m, as in MATLAB). Raising the deck raises the arm.
# Slide: the base frame starts at the mount and moves along +Y; the rail is drawn
# on the deck and the mount post rides on the carriage.

# ---------------------------------------------------------------- platform: chassis, wheels, slope
# The world origin is always on the ground under the ARM MOUNT (the rover's
# frame). Moving the mount moves the chassis and wheels around it.
#   CHASSIS : box from `clearance` to `deck_height` above the ground; `com` is the
#             chassis centre of mass (x, y from the chassis centre; z above ground),
#             not counting the arm. The deck (chassis top) is the platform.
#   ARM_MOUNT: where the arm is mounted on the deck, from the chassis centre (x fwd, y left)
#   WHEELS  : 4 wheels on two side rockers joined by a differential (one rocker
#             tips nose-up while the other tips nose-down). pivot_x: rocker pivot
#             from the chassis centre along x. Wheel contact points are at
#             (+-wheelbase/2, +-track/2) from the chassis centre.
#   SLOPE   : ground slope (deg) and the downhill direction (deg from +X, world).
#             Simulated by tilting gravity (the view stays level).
# >>> AUTO:PLATFORM
CHASSIS = dict(length=1.0, width=0.8, clearance=0.15, deck_height=0.5, mass=45.0, com=(0.0, 0.0, 0.3))
ARM_MOUNT = (0.15, 0.0)
WHEELS = dict(wheelbase=0.8, track=0.7, diameter=0.25, pivot_x=0.0)
SLOPE = dict(deg=0.0, downhill_deg=0.0)
# <<< AUTO:PLATFORM
DRIVE_ACCEL = 1.5          # worst-case braking / accelerating / turning (m/s^2) for the
                           # stability check, on top of the motion the sim produces. PLACEHOLDER
# (placeholders: chassis mass, COM, wheel geometry)

# ---------------------------------------------------------------- link tubes
DENSITY = 2700.0       # kg/m^3 (2700 Al, 7850 steel, ~1600 carbon fibre)
# Rectangular tube per link: width (Y, across the arm), height (in the plane
# of motion), wall. All in mm. wall = 0 -> solid bar.  [MATLAB p.section]
SECTION_MM = {
    "link1":   dict(width=25.4, height=25.4, wall=3.0),
    "link2":   dict(width=25.4, height=25.4, wall=3.0),
    "gripper": dict(width=25.4, height=25.4, wall=3.0),
}
WIRING_MASS = 0.1      # kg, spread by length over link1, link2, gripper [MATLAB m_electronics]
# Other section types per link override SECTION_MM (see sections.py), e.g. the
# braced corner-bar link 1 in arm_params_4bar.py. Empty = all rectangular tubes.
LINK_SECTIONS = {}
# Material for stiffness and stress (E, G in Pa; yield in MPa). 6061-T6 aluminium.
MATERIAL = dict(E=69e9, G=26e9, yield_MPa=276.0)
DESIGN_NAME = "tube link 1"
DEFLECTION_SCALE = 10.0   # how much the drawn bent shape is exaggerated

# ---------------------------------------------------------------- pivot locations
# Where each axis sits relative to the TUBE, in the tube's own axes:
#   x along the tube (near end -> far end), y left (across the arm), z "up"
#   in the plane of motion (perpendicular to the tube).
#   near = this link's own pivot, measured from the tube's NEAR end.
#          x > 0 -> the link rotates from a point along its length
#          (the tube sticks out behind the pivot by x).
#   far  = the NEXT link's pivot, measured from this tube's FAR end.
#          x < 0 -> pivot short of the end; z < 0 -> pivot below the tube.
# The tube length and its tilt relative to the pivot line are worked out so
# the pivot-to-pivot distance stays L1 / L2. y offsets shift the next link
# sideways (they don't change the planar kinematics).
# SHOULDER_PIVOT is the shoulder axis relative to the base centre (carriage / turntable).
# Default layout: link2 runs BESIDE link1, 4 cm to the left (centre to centre:
# 25.4 mm tubes + ~15 mm gap), so the elbow can fold past link1. The gripper is
# in line with link2. The IK allows for the 4 cm sideways offset.
# >>> AUTO:PIVOTS
SHOULDER_PIVOT = (0.0, 0.0, 0.0596)
LINK_GEOMETRY = {
    "link1": dict(near=(0.0, 0.0, 0.0), far=(0.0, 0.0, 0.0)),
    "link2": dict(near=(0.0, -0.04, 0.0), far=(0.0, 0.0, 0.0)),
}
# <<< AUTO:PIVOTS

# ---------------------------------------------------------------- joints
# Pitch/yaw joints: limits in degrees, max effort (N*m), max speed (deg/s).
# base_slide (used when BASE_JOINT = "slide_y"): limits in METRES, effort in N,
# speed in m/s. base_yaw is used when BASE_JOINT = "yaw".
# Effort is enforced in the sim: a motor that is too weak will sag.
JOINTS = {
    #               min    max   effort  speed
    "base_slide":  (-0.35, 0.35, 200.0,  0.15),  # PLACEHOLDER (m, m, N, m/s)
    "base_yaw":    (-180,  180,   40.0,   60),   # PLACEHOLDER
    "shoulder":    (-180,  180,  120.0,   45),   # PLACEHOLDER
    "elbow":       (-180,  180,   80.0,   45),   # PLACEHOLDER
    "wrist_pitch": (-180,  180,   20.0,   90),   # PLACEHOLDER
    "wrist_roll":  (-180,  180,   10.0,   90),   # PLACEHOLDER
}

# ---------------------------------------------------------------- motors and other masses
# Each item is rigidly mounted on a body:
#   body   : "rover"     - fixed to the rover (adds no arm load)
#            "base"      - the carriage (or turntable); moves with the base joint;
#                          origin at its centre, at base height above ground
#            "link1"     - upper arm;  origin at the shoulder axis
#            "link2"     - forearm;    origin at the elbow axis
#            "wrist"     - between wrist pitch and gripper yaw; origin at the wrist axis
#            "gripper"   - origin at the wrist axis, x along the gripper to the TCP
#   anchor : what xyz is measured from (link1 / link2 only; others use "pivot"):
#            "pivot"      - the body's own axis, axes along the pivot line
#            "tube_start" - the tube's near end, tube axes (see pivot locations)
#            "tube_end"   - the tube's far end, tube axes
#   xyz    : centre of mass from the anchor (m). z < 0 -> below the tube.
# Motors also have radius / length (m) for drawing and inertia; their axis is
# parallel to the joint they drive. A motor driving the elbow can sit on link1
# (stator on the upper arm) or on link2, at the joint or remote (belt drive).
# >>> AUTO:MOTORS
MOTORS = {
    "base": dict(drives="base", body="rover", anchor="pivot", xyz=(0.0, -0.45, 0.53), mass=1.0, radius=0.045, length=0.1),  # PLACEHOLDER
    "shoulder": dict(drives="shoulder", body="base", anchor="pivot", xyz=(0.0, -0.06, 0.0596), mass=1.2, radius=0.045, length=0.09),  # PLACEHOLDER
    "elbow": dict(drives="elbow", body="link1", anchor="tube_end", xyz=(0.0, -0.05, 0.0), mass=0.8, radius=0.04, length=0.07),  # PLACEHOLDER
    "wrist_pitch": dict(drives="wrist_pitch", body="link2", anchor="tube_end", xyz=(0.0, 0.04, 0.0), mass=0.4, radius=0.03, length=0.06),  # PLACEHOLDER
    "wrist_roll": dict(drives="gripper", body="wrist", anchor="pivot", xyz=(0.0, 0.0, 0.0), mass=0.3, radius=0.025, length=0.05),  # PLACEHOLDER
}
# <<< AUTO:MOTORS

# Anything else bolted on: gripper actuator, camera, brackets, counterweight.
# Same body / anchor / xyz rules; drawn as boxes of the given size (m).
# >>> AUTO:EXTRAS
EXTRA_MASSES = {
    "carriage": dict(body="base", anchor="pivot", xyz=(0.0, 0.0, -0.02), mass=1.0, size=(0.15, 0.15, 0.04)),  # PLACEHOLDER
    "gripper_mechanism": dict(body="gripper", anchor="pivot", xyz=(0.07, 0.0, 0.0), mass=0.59, size=(0.08, 0.06, 0.06)),  # PLACEHOLDER (gripper tube + this = 0.7 kg)
    "camera": dict(body="gripper", anchor="pivot", xyz=(0.03, 0.0, 0.05), mass=0.1, size=(0.03, 0.06, 0.03)),  # PLACEHOLDER
}
# <<< AUTO:EXTRAS

# Force a body's centre of mass (in that body's frame, m) instead of the value
# computed from its parts, e.g. once you have it from CAD. Mass is unchanged.
COM_OVERRIDE = {
    # "link1": (0.45, 0.0, 0.0),
}

# ---------------------------------------------------------------- gripper (jaws)
# jaw_axis: the gripper-frame axis the jaws close along ("y" = parallel to the
# pitch axes, "z" = in the plane of motion). max_opening: widest jaw gap (m).
GRIPPER = dict(jaw_axis="y", max_opening=0.09)   # PLACEHOLDER until the gripper is designed

# ---------------------------------------------------------------- payload
# Box held at the TCP, as in the MATLAB carry sim. mass = 0 -> empty gripper.
# The box centre sits (size[0]/2 - inset) along the gripper past the TCP.
PAYLOAD = dict(mass=0.0, size=(0.40, 0.40, 0.40), inset=0.0)   # set mass=5.0 for the URC box
# (PAYLOAD is a box permanently fixed to the gripper. The box -> drawer task
#  below uses its own box, so leave PAYLOAD at 0 for the task.)

# ================================================================ TASK: box -> drawer
# World frame = the rover's starting pose (origin on the ground under the arm
# mount, X forward). The rover can drive between mission phases.

# The box: 40 cm envelope including the handle, 5 kg. Its frame is at the
# centre of its bottom face; the handle bar runs along the box's local X.
TASK_BOX = dict(
    size=(0.40, 0.40, 0.40),          # envelope incl. handle (x, y, z), m
    mass=5.0,
    handle=dict(diameter=0.075,       # bar diameter (max per the task), m
                length=0.15,          # bar length, m. PLACEHOLDER
                height=0.10),         # lid -> top of the bar, m. PLACEHOLDER
)
BOX_ZONE = dict(x=(0.60, 1.20), y=(-0.50, 0.50))   # where the box can appear (box centre, world m)

# Drawer cabinet. Its frame: origin at the cabinet's front face, at the tray
# floor height; X = the pull-out direction. pull_deg = 180 -> pulls toward -X
# (toward the rover's start). The tray fits the box with `clearance` per side
# and opens just far enough to clear it ("exact fit").
DRAWER = dict(
    front=(1.70, 0.0),        # front-face centre (x, y), world m. PLACEHOLDER
    floor_z=0.90,             # tray floor height above ground, m. PLACEHOLDER
    pull_deg=180.0,           # pull-out direction (deg from +X, about Z)
    clearance=0.01,           # box-to-wall gap per side, m
    wall=0.015,               # tray wall / floor thickness, m
    wall_height=0.30,         # tray wall height above the floor, m
    panel=0.02,               # front panel thickness, m
    cabinet_depth=0.55,       # behind the front face, m
    handle=dict(diameter=0.03, length=0.12, standoff=0.04, orientation="horizontal"),  # PLACEHOLDER
    mass=3.0,                 # drawer (tray) mass, kg: not loaded on the arm yet
)

# Mission settings
MISSION = dict(
    approach=0.12,            # stand-off before grasping a handle (m)
    lift=0.10,                # box bottom clearance when lifted off the ground (m)
    over_rim=0.05,            # box bottom clearance above the tray rim when moving over it (m)
    place_gap=0.005,          # box bottom above the tray floor when released (m)
    retract=0.15,             # move away after releasing (m)
    linear_speed=0.15,        # straight-line TCP speed (m/s)
    grip_time=0.5,            # pause to close / open the gripper (s)
    rover_speed=(0.30, 30.0), # rover max speed (m/s, deg/s)
    jaw_tol_deg=10.0,         # allowed jaw misalignment with a handle (deg)
    auto_roll=True,           # roll the wrist to square the jaws to each handle, and to line
                              # the box up with the tray (ignores the grasps' roll values)
    reach_step=0.05,          # reach-map grid step (m)
)
ROVER_KE, ROVER_KD = 2.0e4, 2.0e3      # rover position control (N/m, N*s/m; yaw x0.1)

# Saved by "Save task settings" in the sim.
# BOX_POSE: box centre (x, y) on the ground and yaw (deg).
# *_GRASP: gripper pitch (deg, -90 = pointing down) and wrist roll (deg; used when
#          MISSION auto_roll is off).
# ROVER_POSES: rover pose (x, y, yaw_deg) for each phase.
# >>> AUTO:TASK
BOX_POSE = (0.8, 0.0, 0.0)
BOX_GRASP = dict(pitch=-90.0, gripper=0.0)
DRAWER_GRASP = dict(pitch=0.0, gripper=0.0)
ROVER_POSES = dict(open_drawer=(0.0, 0.0, 0.0), pick_up=(0.0, 0.0, 0.0), place=(0.1, 0.0, 0.0))
# <<< AUTO:TASK

# ---------------------------------------------------------------- sim
# Home pose. "base" is metres for the slide, degrees for yaw; the rest are degrees.
HOME_DEG = dict(base=0, shoulder=80, elbow=-150, wrist_pitch=-20, gripper=0)   # folded, gripper down

# Preset buttons in the sim. Each entry is one of:
#   [x, y, z, pitch_deg]   TCP target in world metres (solved with the IK, "Elbow up" setting)
#   dict(joints_deg=[base, shoulder, elbow, wrist_pitch, wrist_roll])   joint values
#        (base in metres for the slide, degrees for yaw)
#   "home" (= HOME_DEG)  or  "straight" (all joints 0)
# NOSE is the rover front (x); DECK is the deck height (z).
NOSE = CHASSIS["length"] / 2 - ARM_MOUNT[0]     # rover front, x from the mount
DECK = CHASSIS["deck_height"]
PRESETS = {
    "Home": "home",
    "Pick up": [NOSE + 0.15, 0.0, 0.40, -90.0],
    "Drawer": [NOSE + 0.15, 0.0, 1.90, -90.0],
    "Straight out": "straight",
}
DRIVE_KE = 5000.0    # position-drive stiffness (N*m/rad; N/m for the slide)
DRIVE_KD = 150.0     # position-drive damping (N*m*s/rad; N*s/m for the slide)
GRAVITY_FF = True    # add each motor's static (gravity) torque to its command, like a real
                     # controller with gravity compensation; removes the sag under load
DESIGN_BOX = 0.15    # half-size (m) of the box the design sliders move within
COLLISION_MARGIN = 0.0   # extra clearance (m) required by the collision check (e.g. 0.01)
URDF_FILE = "urc_arm.urdf"
RECORD_DIR = "recordings"
