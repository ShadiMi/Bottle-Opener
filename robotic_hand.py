"""
robotic_hand.py

Physical utilities and animation functions for PyBullet hand/cap simulation.
Handles hand pose, movement, twisting, vertical alignment, and reset.
"""

import math
import pybullet as p
import time

# Hand joint constants
#TH = Thumb finger, IX=Index finger
TH_MCP = 3
TH_IP  = 5
IX_MCP = 7
IX_PIP = 8
IX_DIP = 9

REST  = {TH_MCP: 0.0,  TH_IP: 0.0,  IX_MCP: 0.0,  IX_PIP: 0.0,  IX_DIP: 0.0}
PINCH = {TH_MCP: 0.30, TH_IP: 0.55, IX_MCP: 0.20, IX_PIP: 0.10, IX_DIP: 1.3}

TIME_STEP = 0.05
FORCE     = 100.0
POS_GAIN  = 1.0

def set_hand_pose(hand, pose):
    """Set hand joints instantly to a pose."""
    for joint, val in pose.items():
        p.setJointMotorControl2(hand, joint, p.POSITION_CONTROL,
            targetPosition=val, positionGain=POS_GAIN, velocityGain=1.0, force=FORCE)

def move_hand(hand, target_pose, step_size=0.015, time_step=TIME_STEP):
    """Smoothly move hand joints to target pose."""
    while p.isConnected():
        done = True
        for joint, target in target_pose.items():
            current = p.getJointState(hand, joint)[0]
            if abs(target - current) > 1e-3:
                done = False
                delta = target - current
                move = max(-step_size, min(step_size, delta))
                p.setJointMotorControl2(hand, joint, p.POSITION_CONTROL,
                    targetPosition=current+move, force=FORCE, positionGain=POS_GAIN, velocityGain=1.0)
            else:
                p.setJointMotorControl2(hand, joint, p.POSITION_CONTROL,
                    targetPosition=target, force=FORCE, positionGain=POS_GAIN, velocityGain=1.0)
        p.stepSimulation()
        time.sleep(time_step)
        if done:
            break

def twist_cap(hand, steps=60, twist_amount=0.3):
    """Simulate twisting motion using thumb and index."""
    start_thumb = p.getJointState(hand, TH_MCP)[0]
    start_index = p.getJointState(hand, IX_MCP)[0]
    for i in range(steps):
        t = i / steps
        thumb_angle = start_thumb + t * twist_amount
        index_angle = start_index - t * twist_amount
        p.setJointMotorControl2(hand, TH_MCP, p.POSITION_CONTROL, targetPosition=thumb_angle, force=FORCE)
        p.setJointMotorControl2(hand, IX_MCP, p.POSITION_CONTROL, targetPosition=index_angle, force=FORCE)
        # Hold rest in pinch
        p.setJointMotorControl2(hand, TH_IP, p.POSITION_CONTROL, targetPosition=PINCH[TH_IP], force=FORCE)
        p.setJointMotorControl2(hand, IX_PIP, p.POSITION_CONTROL, targetPosition=PINCH[IX_PIP], force=FORCE)
        p.setJointMotorControl2(hand, IX_DIP, p.POSITION_CONTROL, targetPosition=PINCH[IX_DIP], force=FORCE)
        p.stepSimulation()
        time.sleep(TIME_STEP)

def lift_hand(hand, cap_body, cap_collision, cap_visual, cap_pos, cap_orn, bottle, cap_link_id, height=0.04, steps=60):
    """
    Lifts the hand and animates the cap popping off and falling.
    """
    # Animate cap popping/falling
    final_pos, final_orn = p.getBasePositionAndOrientation(cap_body)
    p.removeBody(cap_body)
    cap_body_new = p.createMultiBody(
        baseMass=0.02,
        baseCollisionShapeIndex=cap_collision,
        baseVisualShapeIndex=cap_visual,
        basePosition=final_pos,
        baseOrientation=final_orn
    )
    p.changeVisualShape(bottle, cap_link_id, rgbaColor=[1, 1, 1, 0.01])
    p.applyExternalTorque(cap_body_new, -1, [0.03, 0, 0.02], p.WORLD_FRAME)
    p.applyExternalForce(cap_body_new, -1, [3, 0, 0.03], cap_pos, p.WORLD_FRAME)

    # Lift hand
    base_pos, base_orn = p.getBasePositionAndOrientation(hand)
    for i in range(steps):
        t = (i + 1) / steps
        new_z = base_pos[2] + t * height
        new_pos = (base_pos[0], base_pos[1], new_z)
        p.resetBasePositionAndOrientation(hand, new_pos, base_orn)
        set_hand_pose(hand, PINCH)
        p.stepSimulation()
        time.sleep(TIME_STEP)
    # Slide hand after lifting
    for i in range(steps):
        t = (i + 1) / steps
        new_pos2 = (new_pos[0] + t * 0.08, new_pos[1], new_pos[2])
        p.resetBasePositionAndOrientation(hand, new_pos2, base_orn)
        set_hand_pose(hand, PINCH)
        p.stepSimulation()
        time.sleep(TIME_STEP)

def vertical_alignment(hand, size="big", steps=60):
    """Smoothly align the hand Z to 'big' or 'small'."""
    z_targets = {"small": 0.43, "big": 0.51}
    target_z = z_targets.get(size, 0.43)
    base_pos, base_orn = p.getBasePositionAndOrientation(hand)
    start_z = base_pos[2]
    new_pos = list(base_pos)
    for i in range(steps):
        t = (i + 1) / steps
        new_pos[2] = start_z + (target_z - start_z) * t
        p.resetBasePositionAndOrientation(hand, tuple(new_pos), base_orn)
        set_hand_pose(hand, REST)
        p.stepSimulation()
        time.sleep(TIME_STEP)

def reset_hand(hand, steps=60):
    """Smoothly returns the hand to its base position and REST pose."""
    start_pos, start_orn = p.getBasePositionAndOrientation(hand)
    start_euler = p.getEulerFromQuaternion(start_orn)
    target_pos = [0.03, 0.13, 0.43]
    target_euler = [math.pi/2, 0, 0]
    current_joints = [p.getJointState(hand, j)[0] for j in REST]
    target_joints = [REST[j] for j in REST]
    for i in range(steps):
        t = (i + 1) / steps
        interp_pos = [start_pos[j] + (target_pos[j] - start_pos[j]) * t for j in range(3)]
        interp_euler = [start_euler[j] + (target_euler[j] - start_euler[j]) * t for j in range(3)]
        interp_orn = p.getQuaternionFromEuler(interp_euler)
        interp_pose = {joint: current_joints[idx] + (target_joints[idx] - current_joints[idx]) * t
                       for idx, joint in enumerate(REST)}
        set_hand_pose(hand, interp_pose)
        p.resetBasePositionAndOrientation(hand, interp_pos, interp_orn)
        p.stepSimulation()
        time.sleep(TIME_STEP)
