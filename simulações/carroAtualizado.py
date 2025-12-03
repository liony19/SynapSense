import time
import math
import json

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import requests

import pybullet as p
import pybullet_data

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

def send_to_node_red(data_dict):
    try:
        requests.post(NODE_RED_URL, json=data_dict, timeout=0.05)
    except Exception:
        pass

def create_finish_line(y_pos):
    rows = 2
    cols = 10
    size = 0.5
    half = size / 2
    white = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, half, 0.001], rgbaColor=[1,1,1,1])
    black = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, half, 0.001], rgbaColor=[0.1,0.1,0.1,1])
    start_x = -2.5
    for r in range(rows):
        for c in range(cols):
            x = start_x + (c*size) + half
            y = y_pos + (r*size)
            vis = black if ((r+c) % 2 == 0) else white
            p.createMultiBody(0, -1, vis, [x,y,0.002])

def create_env(length=40.0):
    p.loadURDF("plane.urdf")
    track_width = 10.0
    segments = 80

    col_wall = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 1.5, 1])
    vis_wall = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 1.5, 1], rgbaColor=[0.6,0.6,0.6,1])

    for i in range(segments):
        y = (i / segments) * length
        p.createMultiBody(0, col_wall, vis_wall, [track_width/2, y, 1])
        p.createMultiBody(0, col_wall, vis_wall, [-track_width/2, y, 1])

    obs_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.6, length=1.2, rgbaColor=[1,0,0,1])
    obs_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.6, height=1.2)

    obstacles = [
        [-2.0, 4.0],
        [1.5, 8.0],
        [-1.0, 13.0],
        [2.0, 18.0],
        [-2.5, 23.0],
        [0.0, 28.0],
        [2.5, 32.0],
    ]

    for x, y in obstacles:
        p.createMultiBody(0, obs_col, obs_vis, [x, y, 0.6])

    create_finish_line(length - 1.0)

def get_sensors(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    rot = p.getMatrixFromQuaternion(orn)
    fwd = np.array([rot[0], rot[3], rot[6]])
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
        fr.append(start.tolist())
        to.append((start + n*RAY_LENGTH).tolist())
    res = p.rayTestBatch(fr, to)
    d = []
    p.removeAllUserDebugItems()
    for i,r in enumerate(res):
        hit = r[2]
        dist = hit*RAY_LENGTH
        d.append(dist)
        color = [0,1,0] if dist > OBSTACLE_THRESHOLD*1.2 else [1,0,0]
        p.addUserDebugLine(fr[i], to[i], color, 2, 0.15)
    while len(d) < 5: d.append(RAY_LENGTH)
    return tuple(d)

def calculate_motor_speeds(sensors, dt):
    global LAST_ERROR
    d_fl, d_l, d_c, d_r, d_fr = sensors
    if d_c < OBSTACLE_THRESHOLD:
        L = min(d_fl, d_l)
        R = min(d_fr, d_r)
        vb = BASE_SPEED*0.3
        if L > R:
            vL = vb*-0.5
            vR = vb*1.5
        elif R > L:
            vL = vb*1.5
            vR = vb*-0.5
        else:
            vL = -BASE_SPEED*0.5
            vR = -BASE_SPEED*0.5
        lim = BASE_SPEED*1.5
        return float(np.clip(vL,-lim,lim)), float(np.clip(vR,-lim,lim))
    e = d_r - d_l
    der = (e - LAST_ERROR) / (dt if dt>1e-9 else 1e-9)
    LAST_ERROR = e
    steer = KP_BALANCE*e + KD_BALANCE*der
    steer = float(np.clip(steer, -BASE_SPEED, BASE_SPEED))
    vL = BASE_SPEED + steer
    vR = BASE_SPEED - steer
    if d_l > WALL_BALANCE_THRESHOLD and d_r > WALL_BALANCE_THRESHOLD:
        vL *= 1.1
        vR *= 1.1
    vL = float(np.clip(vL, -MAX_SPEED_MOTOR, MAX_SPEED_MOTOR))
    vR = float(np.clip(vR, -MAX_SPEED_MOTOR, MAX_SPEED_MOTOR))
    return vL, vR

def get_joint_indices_by_name(body_id, names):
    m = {}
    n = p.getNumJoints(body_id)
    for i in range(n):
        info = p.getJointInfo(body_id, i)
        m[info[1].decode("utf-8")] = i
    return [m.get(nm, -1) for nm in names]

def main():
    global LAST_ERROR
    urdf = create_mobile_robot_4wd_urdf()
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0,0,-9.8)
    ts = 1/240
    p.setTimeStep(ts)
    p.setPhysicsEngineParameter(fixedTimeStep=ts)
    create_env(TRACK_LENGTH)
    orn = p.getQuaternionFromEuler([0,0,1.57])
    robot = p.loadURDF(urdf, [0,0,0.2], orn, useFixedBase=False)
    wheels = ["fl_joint","fr_joint","rl_joint","rr_joint"]
    idx = get_joint_indices_by_name(robot, wheels)
    MAX_FORCE = 50
    logs = []
    success = False
    start = time.time()
    fc = 0
    try:
        while p.isConnected():
            pos,_ = p.getBasePositionAndOrientation(robot)
            y = pos[1]
            if y > TRACK_LENGTH-2:
                end = time.time()
                send_to_node_red({"status":"completed","total_time": end-start,"final_pos_y":float(y)})
                success=True
                break
            sensors = get_sensors(robot)
            vL, vR = calculate_motor_speeds(sensors, ts)
            vel = [vL, vR, vL, vR]
            for j,v in zip(idx,vel):
                if j>=0:
                    p.setJointMotorControl2(robot,j,p.VELOCITY_CONTROL,targetVelocity=v,force=MAX_FORCE)
            p.stepSimulation()
            p.resetDebugVisualizerCamera(4,0,-60,pos)
            t = time.time()-start
            d_fl,d_l,d_c,d_r,d_fr = sensors
            logs.append({
                "x":float(pos[0]),"y":float(pos[1]),
                "dFL":float(d_fl),"dL":float(d_l),"dC":float(d_c),
                "dR":float(d_r),"dFR":float(d_fr),
                "vL":float(vL),"vR":float(vR),"time":float(t)
            })
            fc+=1
            if fc%10==0:
                send_to_node_red({
                    "timestamp":float(t),
                    "position":{"x":float(pos[0]),"y":float(pos[1])},
                    "sensors":{"FL":float(d_fl),"L":float(d_l),"C":float(d_c),"R":float(d_r),"FR":float(d_fr)},
                    "motors":{"vL":float(vL),"vR":float(vR)},
                    "status":"running"
                })
            time.sleep(0.0)
    finally:
        p.disconnect()

    if logs:
        df = pd.DataFrame(logs)
        df.to_csv("gauntlet_log_retangular_cilindros.csv", index=False)
        plt.figure(figsize=(12,8))
        plt.subplot(2,2,1)
        plt.plot(df["x"],df["y"],"-",linewidth=2)
        plt.title("Trajetória")
        plt.xlabel("X"); plt.ylabel("Y"); plt.axis("equal")

        plt.subplot(2,2,2)
        plt.plot(df["time"],df["dL"],"--")
        plt.plot(df["time"],df["dC"],"-")
        plt.plot(df["time"],df["dR"],"--")
        plt.axhline(OBSTACLE_THRESHOLD,linestyle=":")
        plt.axhline(WALL_BALANCE_THRESHOLD,linestyle=":")
        plt.title("Sensores"); plt.xlabel("Tempo"); plt.ylabel("Distância")

        plt.subplot(2,2,3)
        plt.plot(df["time"],df["vL"],"-")
        plt.plot(df["time"],df["vR"],"-")
        plt.title("Motores"); plt.xlabel("Tempo"); plt.ylabel("Velocidade")

        plt.subplot(2,2,4)
        dist = np.sum(np.sqrt(np.diff(df["x"])**2 + np.diff(df["y"])**2)) if len(df)>1 else 0
        fim = df["time"].iloc[-1] if len(df)>0 else 0
        txt = (
            f"RETILÍNEO + CILINDROS\n"
            f"Tempo: {fim:.1f}s\n"
            f"Distância: {dist:.1f} m\n"
            f"Obstáculos: 7\n"
            f"P={KP_BALANCE}, D={KD_BALANCE}\n"
            f"Status: {'SUCESSO' if success else 'FALHA'}"
        )
        plt.text(0.01,0.5,txt)
        plt.axis("off")
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    main()
