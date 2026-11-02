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
BASE_HEIGHT = 0.55         # top of the carriage / turntable bearing above ground (m). PLACEHOLDER
# Shoulder height = BASE_HEIGHT + SHOULDER_PIVOT z  (0.6096 m in MATLAB).
# Slide: the base frame starts at (0, 0, BASE_HEIGHT) and moves along +Y; the rail
# is drawn on the deck and the mount post rides on the carriage.

# ---------------------------------------------------------------- rover body (drawn; contacts off)
ROVER = dict(
    mount_behind_front=0.35,   # mount is this far behind the nose (m)
    length=1.0,                # nose -> back (m)
    width=0.8,
    ground_clearance=0.15,     # body bottom (m)
    deck_height=0.50,          # body top (m)
)

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
    "gripper_yaw": ( -90,   90,   10.0,   90),   # PLACEHOLDER
}

# ---------------------------------------------------------------- motors and other masses
# Each item is rigidly mounted on a body:
#   body   : "rover"     - fixed to the rover (adds no arm load)
#            "base"      - the carriage (or turntable); moves with the base joint;
#                          origin at its centre, BASE_HEIGHT above ground
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
    "gripper_yaw": dict(drives="gripper_yaw", body="wrist", anchor="pivot", xyz=(0.0, 0.0, 0.0), mass=0.3, radius=0.025, length=0.05),  # PLACEHOLDER
}
# <<< AUTO:MOTORS

# Anything else bolted on: gripper actuator, camera, brackets, counterweight.
# Same body / anchor / xyz rules; drawn as boxes of the given size (m).
# >>> AUTO:EXTRAS
EXTRA_MASSES = {
    "carriage": dict(body="base", anchor="pivot", xyz=(0.0, 0.0, -0.02), mass=1.0, size=(0.15, 0.15, 0.04)),  # PLACEHOLDER
    "gripper_actuator": dict(body="gripper", anchor="pivot", xyz=(0.05, 0.0, 0.03), mass=0.3, size=(0.06, 0.04, 0.04)),  # PLACEHOLDER
    "camera": dict(body="gripper", anchor="pivot", xyz=(0.03, 0.0, 0.05), mass=0.1, size=(0.03, 0.06, 0.03)),  # PLACEHOLDER
}
# <<< AUTO:EXTRAS

# Force a body's centre of mass (in that body's frame, m) instead of the value
# computed from its parts, e.g. once you have it from CAD. Mass is unchanged.
COM_OVERRIDE = {
    # "link1": (0.45, 0.0, 0.0),
}

# ---------------------------------------------------------------- payload
# Box held at the TCP, as in the MATLAB carry sim. mass = 0 -> empty gripper.
# The box centre sits (size[0]/2 - inset) along the gripper past the TCP.
PAYLOAD = dict(mass=0.0, size=(0.40, 0.40, 0.40), inset=0.0)   # set mass=5.0 for the URC box

# ---------------------------------------------------------------- sim
# Home pose. "base" is metres for the slide, degrees for yaw; the rest are degrees.
HOME_DEG = dict(base=0, shoulder=80, elbow=-150, wrist_pitch=-20, gripper_yaw=0)   # folded, gripper down

# Preset buttons in the sim. Each entry is one of:
#   [x, y, z, pitch_deg]   TCP target in world metres (solved with the IK, "Elbow up" setting)
#   dict(joints_deg=[base, shoulder, elbow, wrist_pitch, gripper_yaw])   joint values
#        (base in metres for the slide, degrees for yaw)
#   "home" (= HOME_DEG)  or  "straight" (all joints 0)
# NOSE is the rover front (x); DECK is the deck height (z).
NOSE = ROVER["mount_behind_front"]
DECK = ROVER["deck_height"]
PRESETS = {
    "Home": "home",
    "Pick up": [NOSE + 0.15, 0.0, 0.40, -90.0],
    "Drawer": [NOSE + 0.15, 0.0, 1.90, -90.0],
    "Straight out": "straight",
}
DRIVE_KE = 5000.0    # position-drive stiffness (N*m/rad; N/m for the slide)
DRIVE_KD = 150.0     # position-drive damping (N*m*s/rad; N*s/m for the slide)
DESIGN_BOX = 0.15    # half-size (m) of the box the design sliders move within
COLLISION_MARGIN = 0.0   # extra clearance (m) required by the collision check (e.g. 0.01)
URDF_FILE = "urc_arm.urdf"
RECORD_DIR = "recordings"
