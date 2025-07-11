import os, time, math
import pybullet as p, pybullet_data
import speech_recognition as sr
import trimesh as tm
import threading

os.environ["PYTHONWARNINGS"] = "ignore"
os.environ["SDL_AUDIODRIVER"] = "dsp"

# ---- Hand setup and joint IDs ----
TIME_STEP = 0.05
FORCE     = 100.0
POS_GAIN  = 1.0

SCALE          = 1.8

TH_MCP = 3
TH_IP  = 5
IX_MCP = 7
IX_PIP = 8
IX_DIP = 9

REST  = {TH_MCP: 0.0,  TH_IP: 0.0,  IX_MCP: 0.0,  IX_PIP: 0.0,  IX_DIP: 0.0}
PINCH = {TH_MCP: 0.30, TH_IP: 0.55, IX_MCP: 0.20, IX_PIP: 0.10, IX_DIP: 1.3}
CURRENT_POSE = REST
last_command = None
listening = False

# ---- Load hand (replace with your own loading code/positions) ----
HERE = os.path.dirname(__file__)
RES  = os.path.join(HERE, "resources")
HAND_PKG   = os.path.join(RES, "shadow_hand_ign")
HAND_URDF  = os.path.join(HAND_PKG, "urdf", "shadow_hand.urdf")
BOTTLE_URDF = os.path.join(RES, "bottles", "bottle_500ml.urdf")
BOTTLE_OBJ  = os.path.join(RES, "bottles/meshes/visual/plastic_body.obj")
p.connect(p.GUI)
p.setAdditionalSearchPath(HAND_PKG)
p.setAdditionalSearchPath(RES)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
hand = p.loadURDF(HAND_URDF, basePosition=[0,0.13,0.43],
                  baseOrientation=p.getQuaternionFromEuler([math.pi/2,0,0]), useFixedBase=True)
bottom_z = tm.load(BOTTLE_OBJ).bounds[0, 2]
btl_xyz  = [0.06, -0.24, 0.17]
bottle   = p.loadURDF(BOTTLE_URDF, basePosition=btl_xyz,
                      globalScaling=SCALE, useFixedBase=True)
# ---- Utility to set pose instantly ----
def set_hand_pose(pose):
    for joint, val in pose.items():
        p.setJointMotorControl2(hand, joint, p.POSITION_CONTROL,
            targetPosition=val, positionGain=POS_GAIN, velocityGain=1.0, force=FORCE)

set_hand_pose(REST)

# ---- Smooth transition function ----
def move_hand(target_pose, step_size=0.015, time_step=TIME_STEP):
    while p.isConnected():
        done = True
        for joint, target in target_pose.items():
            current = p.getJointState(hand, joint)[0]
            if abs(target - current) > 1e-3:
                done = False
                # Linear step toward target
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

def twist_cap(steps=60, twist_amount=0.3):
    # twist_amount: total rotation in radians (e.g., 0.4 ≈ 23°)
    start_thumb = p.getJointState(hand, TH_MCP)[0]
    start_index = p.getJointState(hand, IX_MCP)[0]
    for i in range(steps):
        t = i / steps
        # Linear interpolation for smoothness
        thumb_angle = start_thumb + t * twist_amount
        index_angle = start_index - t * twist_amount
        # Hold the rest of the fingers in pinch
        p.setJointMotorControl2(hand, TH_MCP, p.POSITION_CONTROL,
                                targetPosition=thumb_angle, force=FORCE)
        p.setJointMotorControl2(hand, IX_MCP, p.POSITION_CONTROL,
                                targetPosition=index_angle, force=FORCE)
        # Keep other joints at pinch pose
        p.setJointMotorControl2(hand, TH_IP, p.POSITION_CONTROL, targetPosition=PINCH[TH_IP], force=FORCE)
        p.setJointMotorControl2(hand, IX_PIP, p.POSITION_CONTROL, targetPosition=PINCH[IX_PIP], force=FORCE)
        p.setJointMotorControl2(hand, IX_DIP, p.POSITION_CONTROL, targetPosition=PINCH[IX_DIP], force=FORCE)
        p.stepSimulation()
        time.sleep(TIME_STEP)
    print("Twist complete.")

def lift_hand(height=0.06, steps=60):
    # Raise the hand by 'height' meters over 'steps' simulation steps
    base_pos, base_orn = p.getBasePositionAndOrientation(hand)
    target_pos = (base_pos[0], base_pos[1], base_pos[2] + height)
    for i in range(steps):
        t = (i + 1) / steps
        # Linear interpolation
        new_pos = (
            base_pos[0],
            base_pos[1],
            base_pos[2] + t * height
        )
        p.resetBasePositionAndOrientation(hand, new_pos, base_orn)
        # Hold the pinch during the lift
        set_hand_pose(PINCH)
        p.stepSimulation()
        time.sleep(TIME_STEP)

# ---- Voice recognizer setup ----
"""print("Available microphones:")
mics = sr.Microphone.list_microphone_names()
for idx, mic_name in enumerate(mics):
    print(f"{idx}: {mic_name}")"""

recognizer = sr.Recognizer()
# Set device_index=0 or your correct mic index below if necessary!
mic = sr.Microphone()  # mic = sr.Microphone(device_index=0)

def listen_for_command():
    with mic as source:
        print("Say 'rest' or 'pinch'...")
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        try:
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=3)
        except sr.WaitTimeoutError:
            print("Listening timed out, try again.")
            return ""
    try:
        text = recognizer.recognize_google(audio)
        print(f"You said: {text}")
        return text.lower()
    except sr.UnknownValueError:
        print("Didn't catch that.")
        return ""
    except sr.RequestError as e:
        print(f"Speech recognition error: {e}")
        return ""

# ---- Main simulation loop with voice commands ----
print("Simulation running. Say 'rest' or 'open' to move the hand.")

def listen_thread():
    global last_command, listening
    listening = True
    cmd = listen_for_command()   # your function as before
    if cmd:
        last_command = cmd
    listening = False

while p.isConnected():
    set_hand_pose(CURRENT_POSE)
    p.stepSimulation()
    time.sleep(TIME_STEP)
    # Start listening if not already listening
    if not listening:
        t = threading.Thread(target=listen_thread)
        t.daemon = True
        t.start()
    # Handle last_command if received
    if last_command:
        if "open" in last_command:
            print("Switching to PINCH pose.")
            CURRENT_POSE = PINCH
            move_hand(PINCH)
            last_command = None

        elif "twist" in last_command:
            print("Twisting cap")
            twist_cap(steps=60,twist_amount=0.3)
            last_command = None

        elif ("lift" or "left") in last_command:
            print("Lifting cap...")
            lift_hand(height=0.02, steps=60)
            last_command = None

        elif "rest" in last_command:
            print("Switching to REST pose.")
            CURRENT_POSE = REST
            move_hand(REST)
            last_command = None
