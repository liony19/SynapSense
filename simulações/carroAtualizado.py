import time
import math
import json

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import requests
import random

import pybullet as p
import pybullet_data

# =====================================================================
# CONFIGURAÇÕES
# =====================================================================

NODE_RED_URL = "http://localhost:1880/dados-robo"
BASE_SPEED = 20.0
MAX_SPEED_MOTOR = 30.0
OBSTACLE_THRESHOLD = 2.0
WALL_BALANCE_THRESHOLD = 2.5
KP_BALANCE = 10.0
KD_BALANCE = 1.0
LAST_ERROR = 0.0

TRACK_LENGTH = 40.0
RAY_LENGTH = 5.0

# =====================================================================
# URDF DO ROBÔ
# =====================================================================

def create_mobile_robot_4wd_urdf(filename="mobile_robot_4wd.urdf"):
    urdf = """<?xml version="1.0"?>
<robot name="mobile_robot_4wd">
    <material name="rb_navy"><color rgba="0.04 0.08 0.25 1"/></material>
    <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>

    <link name="base_link">
        <visual><geometry><box size="0.5 0.3 0.1"/></geometry><material name="rb_navy"/></visual>
        <collision><geometry><box size="0.5 0.3 0.1"/></geometry></collision>
        <inertial><mass value="10.0"/><inertia ixx="0.1" iyy="0.1" izz="0.1"/></inertial>
    </link>

    <link name="fl_wheel">
        <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="black"/></visual>
        <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
        <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
    </link>
    <joint name="fl_joint" type="continuous">
        <parent link="base_link"/><child link="fl_wheel"/>
        <origin rpy="-1.5708 0 0" xyz="0.15 0.175 -0.05"/><axis xyz="0 0 1"/>
    </joint>

    <link name="fr_wheel">
        <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="black"/></visual>
        <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
        <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
    </link>
    <joint name="fr_joint" type="continuous">
        <parent link="base_link"/><child link="fr_wheel"/>
        <origin rpy="-1.5708 0 0" xyz="0.15 -0.175 -0.05"/><axis xyz="0 0 1"/>
    </joint>

    <link name="rl_wheel">
        <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="black"/></visual>
        <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
        <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
    </link>
    <joint name="rl_joint" type="continuous">
        <parent link="base_link"/><child link="rl_wheel"/>
        <origin rpy="-1.5708 0 0" xyz="-0.15 0.175 -0.05"/><axis xyz="0 0 1"/>
    </joint>

    <link name="rr_wheel">
        <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="black"/></visual>
        <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
        <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
    </link>
    <joint name="rr_joint" type="continuous">
        <parent link="base_link"/><child link="rr_wheel"/>
        <origin rpy="-1.5708 0 0" xyz="-0.15 -0.175 -0.05"/><axis xyz="0 0 1"/>
    </joint>
</robot>
"""
    with open(filename, "w") as f:
        f.write(urdf)
    return filename

# =====================================================================
# NODE-RED
# =====================================================================

def send_to_node_red(data_dict):
    try:
        requests.post(NODE_RED_URL, json=data_dict, timeout=0.03)
    except:
        pass

# =====================================================================
# LINHA DE CHEGADA
# =====================================================================

def create_finish_line(y_pos):
    size = 0.4
    white = p.createVisualShape(p.GEOM_BOX, halfExtents=[size/2,size/2,0.01], rgbaColor=[1,1,1,1])
    black = p.createVisualShape(p.GEOM_BOX, halfExtents=[size/2,size/2,0.01], rgbaColor=[0,0,0,1])

    for i in range(10):
        for j in range(2):
            color = white if (i+j)%2==0 else black
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=color,
                basePosition=[-2+i*size, y_pos+j*size, 0.02]
            )

# =====================================================================
# AMBIENTE OTIMIZADO (PAREDES + OBSTÁCULOS ALEATÓRIOS)
# =====================================================================

def create_env(length=40.0):
    p.loadURDF("plane.urdf")

    track_width = 10.0
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 1.5, 1])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 1.5, 1], rgbaColor=[0.6,0.6,0.6,1])

    for i in range(20):
        y = (i/20)*length
        p.createMultiBody(0, wall_col, wall_vis, [ track_width/2, y, 1])
        p.createMultiBody(0, wall_col, wall_vis, [-track_width/2, y, 1])

    cyl_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.6, height=1.2)
    cyl_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.6, length=1.2, rgbaColor=[1,0,0,1])

    # Geração de Posições Aleatórias
    NUM_OBSTACLES = 10
    SAFE_WIDTH = (track_width / 2) - 0.8 
    START_Y = 5.0
    END_Y = length - 5.0
    
    y_base_positions = np.linspace(START_Y, END_Y, NUM_OBSTACLES)
    
    random_obstacles = []
    for y_base in y_base_positions:
        y_pos = y_base + np.random.uniform(-1.0, 1.0)
        x_pos = np.random.uniform(-SAFE_WIDTH, SAFE_WIDTH)
        
        y_pos = np.clip(y_pos, START_Y, END_Y)
        random_obstacles.append((x_pos, y_pos))

    for x, y in random_obstacles:
        p.createMultiBody(0, cyl_col, cyl_vis, [x, y, 0.6])

    create_finish_line(length-1)

# =====================================================================
# SENSORES
# =====================================================================

def get_sensors(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    rot = p.getMatrixFromQuaternion(orn)

    fwd  = np.array([rot[0], rot[3], rot[6]])
    left = np.array([rot[1], rot[4], rot[7]])

    start = np.array(pos) + np.array([0,0,0.2])

    dirs = [
        fwd + left*1.5,
        fwd + left*0.5,
        fwd,
        fwd - left*0.5,
        fwd - left*1.5,
    ]

    fr = []
    to = []
    for d in dirs:
        n = d / (np.linalg.norm(d)+1e-9)
        fr.append(start)
        to.append(start + n*RAY_LENGTH)

    res = p.rayTestBatch(fr,to)

    return tuple(r[2]*RAY_LENGTH for r in res)

# =====================================================================
# PID + AVOIDANCE
# =====================================================================

def calculate_motor_speeds(sensors, dt):
    global LAST_ERROR
    d_fl, d_l, d_c, d_r, d_fr = sensors

    if d_c < OBSTACLE_THRESHOLD:
        L = min(d_fl,d_l)
        R = min(d_fr,d_r)
        base = BASE_SPEED*0.4
        if L > R:
            return -base, base
        else:
            return base, -base

    e = d_r - d_l
    de = (e - LAST_ERROR) / max(dt,1e-9)
    LAST_ERROR = e
    steer = KP_BALANCE*e + KD_BALANCE*de

    vL = BASE_SPEED + steer
    vR = BASE_SPEED - steer

    return (
        float(np.clip(vL,-MAX_SPEED_MOTOR,MAX_SPEED_MOTOR)),
        float(np.clip(vR,-MAX_SPEED_MOTOR,MAX_SPEED_MOTOR))
    )

# =====================================================================
# FUNÇÃO MAIN
# =====================================================================

def get_joint_indices_by_name(body, names):
    out = {}
    for i in range(p.getNumJoints(body)):
        out[p.getJointInfo(body,i)[1].decode()] = i
    return [out.get(n,-1) for n in names]

def main():
    global LAST_ERROR
    urdf = create_mobile_robot_4wd_urdf()

    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)

    dt = 1/240
    p.setTimeStep(dt)

    create_env(TRACK_LENGTH)

    robot = p.loadURDF(urdf, [0,0,0.2], p.getQuaternionFromEuler([0,0,1.57]))

    idx = get_joint_indices_by_name(robot, ["fl_joint","fr_joint","rl_joint","rr_joint"])

    MAX_FORCE = 40
    logs = []
    success = False

    start = time.time()
    frame = 0
    last_cam = 0

    try:
        while p.isConnected():
            pos,_ = p.getBasePositionAndOrientation(robot)
            if pos[1] > TRACK_LENGTH-2:
                success=True
                send_to_node_red({"status":"completed"})
                break

            sensors = get_sensors(robot)
            vL,vR = calculate_motor_speeds(sensors,dt)

            for j,v in zip(idx,[vL,vR,vL,vR]):
                if j>=0:
                    p.setJointMotorControl2(robot,j,p.VELOCITY_CONTROL,targetVelocity=v,force=MAX_FORCE)

            p.stepSimulation()

            t = time.time()-start
            logs.append({
                "x":pos[0],"y":pos[1],
                "time":t,
                "dL":sensors[1],"dC":sensors[2],"dR":sensors[3],
                "vL":vL,"vR":vR
            })

            if t-last_cam > 1.0:
                p.resetDebugVisualizerCamera(5,0,-60,pos)
                last_cam = t

            if frame%12==0:
                send_to_node_red({"y":float(pos[1]),"status":"running"})

            frame+=1

    finally:
        p.disconnect()

    if logs:
        df = pd.DataFrame(logs)
        df.to_csv("gauntlet_otimizado.csv", index=False)

        plt.figure(figsize=(10,8))
        plt.plot(df["x"],df["y"])
        plt.title("Trajetória")
        plt.xlabel("X"); plt.ylabel("Y")
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    main()