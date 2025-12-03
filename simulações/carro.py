import pybullet as pb
import pybullet_data
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import requests
import math
import json

NODE_ENDPOINT = "http://localhost:1880/robo"

def gerar_urdf_robot():
    urdf_txt = """<?xml version="1.0"?>
<robot name="robot4wd_sim">
  <material name="blue_dark"><color rgba="0.06 0.07 0.20 1"/></material>
  <material name="y_gold"><color rgba="0.70 0.70 0.70 1"/></material>
  <material name="r_bright"><color rgba="1.00 0.50 0.00 1"/></material>
  <material name="w_black"><color rgba="0.05 0.05 0.05 1"/></material>

  <link name="chassis">
    <visual>
      <geometry><box size="0.5 0.3 0.1"/></geometry>
      <material name="blue_dark"/>
    </visual>

    <visual>
      <origin xyz="0.18 0 0.055"/>
      <geometry><box size="0.22 0.15 0.01"/></geometry>
      <material name="y_gold"/>
    </visual>

    <visual>
      <origin xyz="-0.1 0.15 0.055"/>
      <geometry><box size="0.3 0.02 0.01"/></geometry>
      <material name="r_bright"/>
    </visual>

    <visual>
      <origin xyz="-0.1 -0.15 0.055"/>
      <geometry><box size="0.3 0.02 0.01"/></geometry>
      <material name="r_bright"/>
    </visual>

    <collision>
      <geometry><box size="0.5 0.3 0.1"/></geometry>
    </collision>

    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
    </inertial>
  </link>

  <!-- Rodas -->
  <link name="wheel_fl">
    <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="w_black"/></visual>
    <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint_fl" type="continuous">
    <parent link="chassis"/><child link="wheel_fl"/>
    <origin rpy="-1.57 0 0" xyz="0.15 0.18 -0.05"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="wheel_fr">
    <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="w_black"/></visual>
    <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint_fr" type="continuous">
    <parent link="chassis"/><child link="wheel_fr"/>
    <origin rpy="-1.57 0 0" xyz="0.15 -0.18 -0.05"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="wheel_rl">
    <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="w_black"/></visual>
    <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint_rl" type="continuous">
    <parent link="chassis"/><child link="wheel_rl"/>
    <origin rpy="-1.57 0 0" xyz="-0.15 0.18 -0.05"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="wheel_rr">
    <visual><geometry><cylinder length="0.05" radius="0.1"/></geometry><material name="w_black"/></visual>
    <collision><geometry><cylinder length="0.05" radius="0.1"/></geometry></collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint_rr" type="continuous">
    <parent link="chassis"/><child link="wheel_rr"/>
    <origin rpy="-1.57 0 0" xyz="-0.15 -0.18 -0.05"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""

    with open("robot4wd_model.urdf", "w") as arq:
        arq.write(urdf_txt)

def enviar_para_node(data):
    try:
        requests.post(NODE_ENDPOINT, json=data, timeout=0.05)
    except:
        pass

def montar_ambiente(ext=25):
    pb.loadURDF("plane.urdf")

    amp = 1.5
    freq = 0.15
    trechos = 60
    largura = 5.0

    colis = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.1, 0.6, 0.5])
    visu = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.1, 0.6, 0.5], rgbaColor=[0.85, 0.85, 0.85, 1])

    for i in range(trechos):
        y = (i / trechos) * ext
        x_c = amp * math.sin(freq * y)

        ang = math.atan(amp * freq * math.cos(freq * y))
        orient = pb.getQuaternionFromEuler([0, 0, ang])

        pb.createMultiBody(baseMass=0,
               baseCollisionShapeIndex=colis,
               baseVisualShapeIndex=visu,
               basePosition=[x_c + largura/2, y, 0.5],
               baseOrientation=orient)
        pb.createMultiBody(baseMass=0,
               baseCollisionShapeIndex=colis,
               baseVisualShapeIndex=visu,
               basePosition=[x_c - largura/2, y, 0.5],
               baseOrientation=orient)

    obst_visual = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.4, 0.4, 0.5], rgbaColor=[0.2, 0.6, 1.0, 1])
    obst_col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.4, 0.4, 0.5])
    obst_list = [
      [ 1.20,  2.5],
      [-1.10,  5.0],
      [ 0.00,  7.5],
      [ 1.50, 10.0],
      [-1.40, 13.0],
      [ 0.30, 17.0]
    ]

    for desloc, y in obst_list:
        x_base = amp * math.sin(freq * y)
        pb.createMultiBody(baseMass=0,
               baseCollisionShapeIndex=obst_col,
               baseVisualShapeIndex=obst_visual,
               basePosition=[x_base + desloc, y, 0.5])

    gerar_linha_chegada(ext - 1)


def gerar_linha_chegada(y):
    tam = 0.5
    metade = tam / 2

    branco = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[metade, metade, 0.001], rgbaColor=[1, 1, 1, 1])
    preto = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[metade, metade, 0.001], rgbaColor=[0.1, 0.1, 0.1, 1])

    inicio_x = -2.5

    for r in range(2):
        for c in range(10):
            x = inicio_x + c * tam + metade
            y_final = y + r * tam
            cor = preto if (r + c) % 2 == 0 else branco
            pb.createMultiBody(baseVisualShapeIndex=cor, basePosition=[x, y_final, 0.002])

def ler_sensores(rb):
    pos, ori = pb.getBasePositionAndOrientation(rb)
    m = pb.getMatrixFromQuaternion(ori)

    frente = np.array([m[0], m[3], m[6]])
    lado = np.array([m[1], m[4], m[7]])

    origem = np.array(pos) + np.array([0, 0, 0.2])
    alcance = 5.0

    dirs = [frente + lado,
            frente + lado * 0.5,
            frente,
            frente - lado * 0.5,
            frente - lado]

    finais = [origem + d * alcance for d in dirs]

    testes = pb.rayTestBatch([origem]*5, finais)

    pb.removeAllUserDebugItems()
    distancias = []

    for i, res in enumerate(testes):
        fator = res[2]
        dist = fator * alcance
        distancias.append(dist)

        cor = [0, 1, 0] if dist > 1.8 else [1, 0, 0]
        pb.addUserDebugLine(origem, origem + (finais[i] - origem)*fator, cor, 2)

    return min(distancias[0], distancias[1]), distancias[2], min(distancias[3], distancias[4])

def executar_sim():
    gerar_urdf_robot()

    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.resetSimulation()
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    pb.setGravity(0, 0, -9.8)

    pista_comp = 22
    montar_ambiente(pista_comp)

    orient_ini = pb.getQuaternionFromEuler([0, 0, 1.57])
    robot = pb.loadURDF("robot4wd_model.urdf",
                        basePosition=[0, 0, 0.2],
                        baseOrientation=orient_ini)

    velocidade_base = 30
    ganho = 8
    limite_obs = 2
    limite_parede = 1.8

    print("Simulação iniciada - Versão reescrita com Node-RED")

    registros = []
    chegou = False
    inicio = time.time()

    envio_tick = 0

    while pb.isConnected():
        pos, _ = pb.getBasePositionAndOrientation(robot)
        y_atual = pos[1]

        if y_atual > pista_comp - 2:
            dur = time.time() - inicio
            print(f"Chegada concluída em {dur:.1f}s")
            chegou = True

            enviar_para_node({
                "status": "done",
                "time_total": dur,
                "pos_y": y_atual
            })
            break

        distL, distC, distR = ler_sensores(robot)

        if distC < limite_obs:
            if distL > distR + 0.3:
                velL, velR = -3, 12
            else:
                velL, velR = 12, -3
        else:
            if distL < limite_parede or distR < limite_parede:
                erro = np.clip(distR - distL, -2.5, 2.5)
                corr = erro * ganho
                velL, velR = velocidade_base + corr, velocidade_base - corr
            else:
                impulso = 1.3
                velL = velocidade_base * impulso
                velR = velocidade_base * impulso

        velL = np.clip(velL, -25, 25)
        velR = np.clip(velR, -25, 25)

        pb.setJointMotorControlArray(robot, [0,2], pb.VELOCITY_CONTROL,
                                     targetVelocities=[velL, velL], forces=[50,50])
        pb.setJointMotorControlArray(robot, [1,3], pb.VELOCITY_CONTROL,
                                     targetVelocities=[velR, velR], forces=[50,50])

        pb.stepSimulation()

        pb.resetDebugVisualizerCamera(4, 0, -60, pos)

        tempo_atual = time.time() - inicio

        registros.append({
            "x": pos[0], "y": pos[1],
            "L": distL, "C": distC, "R": distR,
            "vL": velL, "vR": velR,
            "t": tempo_atual
        })

        envio_tick += 1
        if envio_tick % 10 == 0:
            enviar_para_node({
                "timestamp": tempo_atual,
                "pos": {"x": pos[0], "y": pos[1]},
                "sens": {"L": float(distL), "C": float(distC), "R": float(distR)},
                "mot": {"L": float(velL), "R": float(velR)},
                "status": "run"
            })

    pb.disconnect()

    if chegou:
        df = pd.DataFrame(registros)
        df.to_csv("log_sinuoso_reescrito.csv", index=False)

        plt.figure(figsize=(15, 10))

        plt.subplot(2,2,1)
        plt.plot(df['x'], df['y'], 'b-', lw=2)
        plt.title("Trajetória do Robô")
        plt.axis("equal")

        plt.subplot(2,2,2)
        plt.plot(df['t'], df['L'], 'g--', label="Esq")
        plt.plot(df['t'], df['C'], 'r-', label="Centro")
        plt.plot(df['t'], df['R'], 'b--', label="Dir")
        plt.axhline(limite_obs, color="orange", ls=":", label="Limite Obst.")
        plt.legend()
        plt.title("Sensores")

        plt.subplot(2,2,3)
        plt.plot(df['t'], df['vL'], label="vL")
        plt.plot(df['t'], df['vR'], label="vR")
        plt.title("Velocidades")
        plt.legend()

        plt.subplot(2,2,4)
        total_dist = np.sum(np.sqrt(np.diff(df['x'])**2 + np.diff(df['y'])**2))
        txt = f"""SIMULAÇÃO - VERSÃO ALTERADA

Tempo total: {df['t'].iloc[-1]:.1f}s
Distância percorrida: {total_dist:.1f}m
Obstáculos: 6
"""
        plt.text(0.1, 0.5, txt)
        plt.axis("off")

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    executar_sim()
