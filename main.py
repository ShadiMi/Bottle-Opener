"""
main.py

Main simulation setup, loop, and interaction.
Handles PyBullet world, robot hand, bottle/cap, and command loop.
"""

import os, math, time
import pybullet as p, pybullet_data
import trimesh as tm

from robotic_hand import (
    set_hand_pose, move_hand, twist_cap, lift_hand, vertical_alignment, reset_hand,
    REST, PINCH, TIME_STEP
)
from voice_utils import VoiceCommandListener

os.environ["PYTHONWARNINGS"] = "ignore"
os.environ["SDL_AUDIODRIVER"] = "dsp"

CURRENT_POSE = REST

# ---- Load models and environment ----
HERE = os.path.dirname(__file__)
RES  = os.path.join(HERE, "resources")
HAND_PKG   = os.path.join(RES, "shadow_hand_ign")
HAND_URDF  = os.path.join(HAND_PKG, "urdf", "shadow_hand.urdf")
BOTTLE_URDF = os.path.join(RES, "bottles", "bottle_500ml.urdf")
BOTTLE_OBJ  = os.path.join(RES, "bottles/meshes/visual/plastic_body.obj")
CAP_OBJ = os.path.join(RES, "bottles/meshes/visual/plastic_cap.obj")

p.connect(p.GUI)
p.setAdditionalSearchPath(HAND_PKG)
p.setAdditionalSearchPath(RES)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
hand = p.loadURDF(HAND_URDF, basePosition=[0.03,0.13,0.43],
                  baseOrientation=p.getQuaternionFromEuler([math.pi/2,0,0]), useFixedBase=True)
bottle   = p.loadURDF(BOTTLE_URDF, basePosition=[0.06, -0.24, 0.17], globalScaling=1.8, useFixedBase=True)
bottle_big   = p.loadURDF(BOTTLE_URDF, basePosition=[-0.2, -0.24, 0.17], globalScaling=2.5, useFixedBase=True)

cap_link_id = 0
cap_pos, cap_orn = p.getLinkState(bottle, cap_link_id)[:2]
cap_visual = p.createVisualShape(p.GEOM_MESH, fileName=CAP_OBJ, meshScale=[1.8]*3)
cap_collision = p.createCollisionShape(p.GEOM_MESH, fileName=CAP_OBJ, meshScale=[1.8]*3)
cap_body = p.createMultiBody(
    baseMass=0.0,
    baseCollisionShapeIndex=cap_collision,
    baseVisualShapeIndex=cap_visual,
    basePosition=cap_pos,
    baseOrientation=cap_orn
)

# Remove all hand/bottle collision
p.setCollisionFilterGroupMask(hand, -1, 0, 0)
for j in range(p.getNumJoints(hand)):
    p.setCollisionFilterGroupMask(hand, j, 0, 0)
p.setCollisionFilterGroupMask(bottle, -1, 0, 0)
for j in range(p.getNumJoints(bottle)):
    p.setCollisionFilterGroupMask(bottle, j, 0, 0)

set_hand_pose(hand, REST)

# ---- Voice command setup ----
listener = VoiceCommandListener()

def open_bottle_sequence():
    set_hand_pose(hand, PINCH)
    move_hand(hand, PINCH)
    time.sleep(TIME_STEP)
    print("Twisting cap")
    twist_cap(hand, steps=20, twist_amount=0.4)
    time.sleep(TIME_STEP/10)
    print("Lifting cap...")
    lift_hand(hand, cap_body, cap_collision, cap_visual, cap_pos, cap_orn, bottle, cap_link_id, height=0.04, steps=60)
    time.sleep(TIME_STEP)
    move_hand(hand, REST)

print("Simulation running. Say 'rest', 'open', 'big', 'small', or 'reset'...")

while p.isConnected():
    set_hand_pose(hand, CURRENT_POSE)
    p.stepSimulation()
    time.sleep(TIME_STEP)
    # Start listening if not already listening
    if not listener.listening:
        listener.listen_thread()
    # Handle last_command if received
    if listener.last_command:
        cmd = listener.last_command
        if "open" in cmd:
            print("Opening bottle.")
            open_bottle_sequence()
            CURRENT_POSE = REST
            listener.last_command = None

        elif "rest" in cmd:
            print("Switching to REST pose.")
            CURRENT_POSE = REST
            move_hand(hand, REST)
            listener.last_command = None

        elif "big" in cmd:
            print("big bottle height")
            vertical_alignment(hand, "big", 60)
            CURRENT_POSE = REST
            listener.last_command = None

        elif "small" in cmd:
            print("small bottle height")
            vertical_alignment(hand, "small", 60)
            CURRENT_POSE = REST
            listener.last_command = None

        elif "reset" in cmd:
            reset_hand(hand, 60)
            CURRENT_POSE = REST
            listener.last_command = None

    time.sleep(0.01)
