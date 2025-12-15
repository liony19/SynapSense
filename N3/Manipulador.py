#!/usr/bin/env python3

import time
import math
import random
import json
import requests
import numpy as np
import pybullet as p
import pybullet_data
import sys
import traceback

SIM_STEP = 1.0 / 240.0
SIM_TIME_LIMIT = 180.0
TARGET_VISIBLE_TIME = 9.0
WAIT_AFTER_REACH = 1.0
TARGET_TOLERANCE = 0.03
IK_MAX_ITER = 200

# CONFIGURAÇÃO DO DELAY (Em segundos)
DELAY_BEFORE_RESET = 1.0  

KP = 50.0
KD = 3.0
KI = 0.0
TORQUE_LIMIT = 120.0

TRAJECTORY_LOG_LIMIT = 50000
NODE_RED_URL = "http://localhost:1880/manipulador_log"

try:
    cid = p.connect(p.GUI)
    if cid < 0:
        raise RuntimeError("Falha conexao GUI.")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIM_STEP)
    p.setRealTimeSimulation(0) # Estamos em modo passo-a-passo manual

    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0], useFixedBase=True)

except Exception as e:
    sys.exit(1)

JOINT_INDICES = []
JOINT_NAMES = []
LOWER_LIMITS = []
UPPER_LIMITS = []
JOINT_RANGES = []
REST_POSES = []
EE_LINK_INDEX = -1

for j in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, j)
    if info[2] == p.JOINT_REVOLUTE:
        JOINT_INDICES.append(j)
        JOINT_NAMES.append(str(info[1]))

JOINT_INDICES = JOINT_INDICES[:6]
N_JOINTS = len(JOINT_INDICES)
EE_LINK_INDEX = JOINT_INDICES[-1] + 1

for jidx in JOINT_INDICES:
    info = p.getJointInfo(robot, jidx)
    lower = info[8]
    upper = info[9]
    if lower == upper and lower == 0.0:
        lower = -math.pi
        upper = math.pi
    LOWER_LIMITS.append(float(lower))
    UPPER_LIMITS.append(float(upper))
    JOINT_RANGES.append(float(upper - lower))
    REST_POSES.append(0.0)

for j in range(p.getNumJoints(robot)):
    p.setJointMotorControl2(robot, j, controlMode=p.VELOCITY_CONTROL, force=0)

for j in JOINT_INDICES:
    p.resetJointState(robot, j, 0.0)

# Raio do alvo
p.stepSimulation()
st = p.getLinkState(robot, EE_LINK_INDEX, computeForwardKinematics=True)
R = np.linalg.norm(st[4]) * 0.8

def is_connected():
    try:
        return p.isConnected()
    except Exception:
        return False

def send_log(payload):
    try:
        requests.post(NODE_RED_URL, json=payload, timeout=0.05)
    except Exception:
        pass

def sample_point_in_sphere(radius):
    while True:
        u = random.random()
        costheta = random.uniform(-1, 1)
        phi = random.uniform(0, 2 * math.pi)
        r = radius * (u ** (1/3))
        
        if r < 0.35: 
            continue
            
        theta = math.acos(costheta)
        x = r * math.sin(theta) * math.cos(phi)
        y = r * math.sin(theta) * math.sin(phi)
        z = r * math.cos(theta)
        
        final_z = max(z, 0.1)
        return [x, y, final_z]

def get_ee_pos():
    st = p.getLinkState(robot, EE_LINK_INDEX, computeForwardKinematics=True)
    return list(st[4])

def get_ee_pos_from_joint_positions(joint_positions):
    state_id = None
    try:
        state_id = p.saveState()
        for i, jidx in enumerate(JOINT_INDICES):
            p.resetJointState(robot, jidx, joint_positions[i])
        st = p.getLinkState(robot, EE_LINK_INDEX, computeForwardKinematics=True)
        pos = list(st[4])
    except Exception:
        st = p.getLinkState(robot, EE_LINK_INDEX, computeForwardKinematics=True)
        pos = list(st[4])
    finally:
        if state_id is not None:
            try:
                p.restoreState(state_id)
                p.removeState(state_id)
            except Exception:
                pass
    return pos

def full_joint_vector_from_controlled(ctrl_list):
    num = p.getNumJoints(robot)
    full = [0.0] * num
    for i, jidx in enumerate(JOINT_INDICES):
        full[jidx] = float(ctrl_list[i]) if i < len(ctrl_list) else 0.0
    return full

integral = np.zeros(N_JOINTS)
prev_error = np.zeros(N_JOINTS)

def compute_pid(desired, current):
    global integral, prev_error
    error = np.array(desired) - np.array(current)
    derivative = (error - prev_error) / SIM_STEP
    integral += error * SIM_STEP
    prev_error = error
    torque = KP * error + KD * derivative + KI * integral
    torque = np.clip(torque, -TORQUE_LIMIT, TORQUE_LIMIT)
    return torque.tolist(), float(np.mean(np.abs(error)))

start_time = time.time()
sim_time = 0.0
target_counter = 0
targets_detected = 0
targets_reached = 0
trajectory_log = []
current_target = None
current_marker = None
point_visible = False
point_timestamp = 0.0
overshoot = np.zeros(N_JOINTS)
energy_accum = 0.0
stable_window = 0.1
stable_steps_needed = max(1, int(stable_window / SIM_STEP))
stable_counter = 0
ik_error = 0.0
ik_iter_count = 0
stability_time = 0.0
last_target_time = 0.0

def clamp_angles(q_list):
    out = []
    for i, qv in enumerate(q_list):
        l = LOWER_LIMITS[i]
        u = UPPER_LIMITS[i]
        out.append(float(max(min(qv, u), l)))
    return out

def is_valid_ik_result(q_list):
    if q_list is None: return False
    arr = np.array(q_list)
    if np.any(np.isnan(arr)) or np.any(np.isinf(arr)): return False
    if len(q_list) < N_JOINTS: return False
    return True

try:
    while sim_time < SIM_TIME_LIMIT:
        if not is_connected():
            break

        # --- LÓGICA DE TRANSIÇÃO DE ALVOS ---
        if not point_visible:
            
            # 1. DELAY (PAUSA ESTÁTICA) ANTES DO RESET
            # Mantém a posição atual rígida
            if target_counter > 0: # Só faz delay se não for o primeiro ponto
                print(f"Aguardando {DELAY_BEFORE_RESET}s antes de resetar...")
                current_q_delay = [p.getJointState(robot, j)[0] for j in JOINT_INDICES]
                
                # Trava o robô na posição atual
                p.setJointMotorControlArray(robot, JOINT_INDICES, controlMode=p.POSITION_CONTROL,
                                            targetPositions=current_q_delay, forces=[TORQUE_LIMIT] * N_JOINTS)
                
                delay_end = sim_time + DELAY_BEFORE_RESET
                while sim_time < delay_end:
                    if not is_connected(): break
                    p.stepSimulation()
                    sim_time += SIM_STEP
                    # IMPORTANTE: sleep para respeitar o tempo real visualmente
                    time.sleep(SIM_STEP) 

            # 2. MOVIMENTO DE RESET
            print("Resetando para posição Zero...")
            reset_pose = [0.0] * N_JOINTS
            p.setJointMotorControlArray(robot, JOINT_INDICES, controlMode=p.POSITION_CONTROL,
                                        targetPositions=reset_pose, forces=[TORQUE_LIMIT] * N_JOINTS)
            
            reset_duration = 1.5
            reset_end_time = sim_time + reset_duration
            while sim_time < reset_end_time:
                if not is_connected(): break
                p.stepSimulation()
                sim_time += SIM_STEP
                # IMPORTANTE: sleep para ver o movimento suave
                time.sleep(SIM_STEP) 
            
            # Libera motores para modo de torque
            p.setJointMotorControlArray(robot, JOINT_INDICES, controlMode=p.VELOCITY_CONTROL,
                                        targetVelocities=[0] * N_JOINTS, forces=[0] * N_JOINTS)
            
            # 3. GERAR NOVO ALVO
            current_target = sample_point_in_sphere(R)
            target_counter += 1
            point_timestamp = sim_time
            point_visible = True
            targets_detected += 1
            last_target_time = sim_time

            print(f"Novo alvo gerado: {target_counter}")

            try:
                shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.035, rgbaColor=[1, 0, 0, 0.8])
                current_marker = p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape, basePosition=current_target)
            except Exception:
                current_marker = None

            # Reset de variáveis de controle
            integral[:] = 0.0
            prev_error[:] = 0.0
            overshoot[:] = 0.0
            stable_counter = 0
            stability_time = 0.0
            ik_error = 0.0
            ik_iter_count = 0

        # --- CHECAGEM DE TEMPO VISÍVEL (TIMEOUT) ---
        if sim_time - point_timestamp > TARGET_VISIBLE_TIME:
            print("Alvo expirou (Timeout).")
            point_visible = False
            if current_marker is not None:
                try:
                    p.removeBody(current_marker)
                except Exception:
                    pass
                current_marker = None
            
            # Para o movimento imediatamente
            p.setJointMotorControlArray(robot, JOINT_INDICES, controlMode=p.VELOCITY_CONTROL,
                                            targetVelocities=[0] * N_JOINTS, forces=[0] * N_JOINTS)
            p.stepSimulation()
            sim_time += SIM_STEP
            continue

        # --- SIMULAÇÃO FÍSICA E CONTROLE DO BRAÇO ---
        states = [p.getJointState(robot, j) for j in JOINT_INDICES]
        q = [s[0] for s in states]
        dq = [s[1] for s in states]
        ee = get_ee_pos()
        
        if current_target is None:
            # Fallback caso alvo seja nulo
            current_target = sample_point_in_sphere(R)
        
        dist = math.dist(ee, current_target)

        # Cinemática Inversa
        ik_result = None
        desired_q = q.copy()
        try:
            ik_result_raw = p.calculateInverseKinematics(
                bodyUniqueId=robot,
                endEffectorLinkIndex=EE_LINK_INDEX,
                targetPosition=current_target,
                lowerLimits=LOWER_LIMITS,
                upperLimits=UPPER_LIMITS,
                jointRanges=JOINT_RANGES,
                restPoses=REST_POSES,
                maxNumIterations=IK_MAX_ITER
            )
            if ik_result_raw is not None and len(ik_result_raw) >= N_JOINTS:
                ik_result = [ik_result_raw[i] for i in range(N_JOINTS)]

            if is_valid_ik_result(ik_result):
                desired_q = ik_result
            else:
                desired_q = q.copy()
        except Exception:
            desired_q = q.copy()

        ik_iter_count = IK_MAX_ITER if ik_result is not None else 0
        desired_q = clamp_angles(desired_q)

        # Cálculo de erro IK para log
        ik_error = 0.0
        try:
            ik_ee_pos = get_ee_pos_from_joint_positions(desired_q)
            ik_error = math.dist(ik_ee_pos, current_target)
        except Exception:
            pass

        # Compensação de Gravidade
        gravity_torques = [0.0] * N_JOINTS
        try:
            full_q = full_joint_vector_from_controlled(q)
            zero_list = [0.0] * p.getNumJoints(robot)
            gravity_torques_raw = p.calculateInverseDynamics(
                bodyUniqueId=robot,
                objPositions=full_q,
                objVelocities=zero_list,
                objAccelerations=zero_list
            )
            gravity_torques = [float(x) for x in gravity_torques_raw[:N_JOINTS]]
        except Exception:
            pass

        # Controle PID
        pid_torque, avg_angular_error = compute_pid(desired_q, q)
        total_torque = np.clip(np.array(gravity_torques) + np.array(pid_torque), -TORQUE_LIMIT, TORQUE_LIMIT)

        try:
            p.setJointMotorControlArray(robot, JOINT_INDICES, p.TORQUE_CONTROL, forces=total_torque.tolist())
        except Exception:
            for i, jidx in enumerate(JOINT_INDICES):
                p.setJointMotorControl2(robot, jidx, p.TORQUE_CONTROL, force=float(total_torque[i]))

        energy_accum += float(sum(np.abs(total_torque))) * SIM_STEP
        overshoot = np.maximum(overshoot, np.abs(np.array(q) - np.array(desired_q)))
        
        if len(trajectory_log) < TRAJECTORY_LOG_LIMIT:
            trajectory_log.append(ee)

        # Checagem de Estabilidade (Alvo Alcançado)
        if dist <= TARGET_TOLERANCE:
            if stable_counter == 0:
                stability_time = sim_time - last_target_time
            stable_counter += 1
        else:
            stable_counter = 0
            stability_time = 0.0

        if stable_counter >= stable_steps_needed:
            targets_reached += 1
            print(f"Alvo {target_counter} OK em {stability_time:.2f}s")
            
            # Segura no alvo por 1s (WAIT_AFTER_REACH)
            wait_until = sim_time + WAIT_AFTER_REACH
            q_at_reach = q.copy()
            p.setJointMotorControlArray(robot, JOINT_INDICES, controlMode=p.POSITION_CONTROL,
                                        targetPositions=q_at_reach, forces=[TORQUE_LIMIT] * N_JOINTS,
                                        positionGains=[0.8] * N_JOINTS, velocityGains=[0.5] * N_JOINTS)
            
            while sim_time < wait_until:
                if not is_connected(): break
                p.stepSimulation()
                sim_time += SIM_STEP
                # Sleep opcional aqui também se quiser ver ele parado no alvo
                # time.sleep(SIM_STEP) 

            point_visible = False
            if current_marker is not None:
                try:
                    p.removeBody(current_marker)
                except Exception:
                    pass
                current_marker = None
            continue

        # Envio de Logs para Node-RED (a cada ~0.05s de tempo de simulação)
        if int(sim_time * 20) != int((sim_time - SIM_STEP) * 20):
            payload = {
                "time": sim_time,
                "target": current_target,
                "ee": ee,
                "dist_ee_error": dist,
                "q": q,
                "dq": dq,
                "desired_q": desired_q,
                "mode": "TORQUE_PID",
                "energy_accum": energy_accum,
                "overshoot_max": overshoot.tolist(),
                "avg_angular_error": avg_angular_error,
                "time_to_stabilize": stability_time if stable_counter > 0 else -1.0,
                "ik_error": ik_error,
                "ik_iterations": ik_iter_count,
                "target_visible": point_visible,
                "targets_detected": targets_detected,
                "targets_reached": targets_reached
            }
            send_log(payload)

        if not is_connected(): break
        p.stepSimulation()
        sim_time += SIM_STEP

except KeyboardInterrupt:
    print("Interrompido.")
except Exception:
    traceback.print_exc()
finally:
    print("===")
    print(f"Tempo: {sim_time:.2f} s")
    print(f"Detectados: {targets_detected}")
    print(f"Alcancados: {targets_reached}")
    print(f"Energia: {energy_accum:.2f}")
    print("===")

    if is_connected():
        try:
            p.setJointMotorControlArray(robot, JOINT_INDICES, controlMode=p.TORQUE_CONTROL, forces=[0] * N_JOINTS)
            while is_connected():
                p.stepSimulation()
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
    try:
        if is_connected(): p.disconnect()
    except Exception:
        pass