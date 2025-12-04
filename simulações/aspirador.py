import pybullet as p
import pybullet_data
import time, math, numpy as np, os, random
import requests # Adicionado para comunicação HTTP com Node-RED

# =====================================================================
# CONFIGURAÇÕES E NODE-RED
# =====================================================================

NODE_RED_URL = "http://localhost:1880/dados-robo"
URDF_NAME = "mobile_robot_4wd.urdf"

# Variáveis globais para o ambiente
OBST_IDS = []
OBST_DATA = []
DIRT_IDS = []
DIRT_TOTAL_COUNT = 0 # Inicializado aqui, definido em build_env

# Parâmetros de controle (do código original)
LINEAR_SPEED = 0.8        
COLLISION_DISTANCE = 0.5  
ROTATION_SPEED = 0.05
MAX_STEPS = 3600 # Limite de tempo de simulação

def send_to_node_red(data_dict):
    """Envia dados para o Node-RED via HTTP POST."""
    try:
        requests.post(NODE_RED_URL, json=data_dict, timeout=0.05)
    except:
        pass

# =====================================================================
# URDF DO ROBÔ
# =====================================================================

def ensure_urdf():
    """
    Garante que o arquivo URDF mínimo para o robô exista. 
    Se não existir, cria um arquivo URDF simples para uma caixa.
    """
    if not os.path.exists(URDF_NAME):
        urdf = """<?xml version='1.0'?><robot name='mini'><link name='base'><visual><geometry><box size='0.4 0.3 0.1'/></geometry></visual><collision><geometry><box size='0.4 0.3 0.1'/></geometry></collision></link></robot>"""
        with open(URDF_NAME,'w') as f: f.write(urdf)

# =====================================================================
# AMBIENTE OTIMIZADO (paredes + cilindros + sujeira)
# =====================================================================

def build_env():
    """
    Configura o ambiente de simulação carregando o chão, adicionando obstáculos e criando as paredes.
    """
    global OBST_IDS, OBST_DATA, DIRT_IDS, DIRT_TOTAL_COUNT
    
    p.resetSimulation()
    p.loadURDF('plane.urdf')
    
    wall_color = [0.3, 0.7, 0.7, 1]
    
    def create_wall(pos, extents):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=extents, rgbaColor=wall_color)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=pos)
        
    create_wall(pos=[-7.0, 7.0, 1.0], extents=[0.1, 10.0, 1.0])
    create_wall(pos=[7.0, 7.0, 1.0], extents=[0.1, 10.0, 1.0])
    create_wall(pos=[0.0, -3.0, 1.0], extents=[7.1, 0.1, 1.0])
    create_wall(pos=[0.0, 17.0, 1.0], extents=[7.1, 0.1, 1.0])
    
    OBST_IDS = []
    OBST_DATA = []
    for i in range(20): 
        x = np.random.uniform(-6,6)
        y = np.random.uniform(-2,16) 
        vis_radius = 0.2
        vis_height = 1.0
        col_radius = 0.18
        col_height = 0.95
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=col_radius, height=col_height)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=vis_radius, length=vis_height, rgbaColor=[0.3, 0.7, 0.3, 1])
        oid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=[x,y,vis_height*0.5])
        OBST_IDS.append(oid)
        OBST_DATA.append((x, y, col_radius))

    DIRT_IDS = []
    dirt_count = 25
    DIRT_TOTAL_COUNT = dirt_count # Define o total para o Node-RED
    attempts = 0
    placed = 0
    margin_from_wall = 0.3
    dirt_radius = 0.05
    while placed < dirt_count and attempts < dirt_count * 50:
        attempts += 1
        dx = np.random.uniform(-7.0+margin_from_wall, 7.0-margin_from_wall)
        dy = np.random.uniform(-3.0+margin_from_wall, 17.0-margin_from_wall)
        too_close = False
        for ox, oy, orad in OBST_DATA:
            if math.hypot(dx-ox, dy-oy) < (orad + dirt_radius + 0.2):
                too_close = True
                break
        if too_close:
            continue
        d_col = p.createCollisionShape(p.GEOM_SPHERE, radius=dirt_radius)
        d_vis = p.createVisualShape(p.GEOM_SPHERE, radius=dirt_radius, rgbaColor=[0.9, 0.85, 0.1, 1])
        did = p.createMultiBody(baseMass=0.01, baseCollisionShapeIndex=d_col, baseVisualShapeIndex=d_vis, basePosition=[dx, dy, dirt_radius])
        try:
            p.setCollisionFilterGroupMask(did, -1, 0, 0)
        except Exception:
            pass
        DIRT_IDS.append(did)
        placed += 1

# =====================================================================
# SENSORES
# =====================================================================

def simple_lidar(robot_id, n=9, ray_len=2.0):
    """
    Simula um sensor LiDAR simples (sensor de distância por raios).
    n: número de raios (9 por padrão, abrangendo -90 a 90 graus).
    ray_len: comprimento máximo do raio.
    """
    pos, quat = p.getBasePositionAndOrientation(robot_id)
    start = [pos[0], pos[1], pos[2]+0.2]
    yaws = np.linspace(-math.pi/2, math.pi/2, n)
    dists = []
    
    rot_matrix = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
    
    for a in yaws:
        local_dir = np.array([math.sin(a) * ray_len, math.cos(a) * ray_len, 0])
        
        global_dir = np.dot(rot_matrix, local_dir)
        
        end = start + global_dir
        
        res = p.rayTest(start, end)[0]
        dists.append(res[2] * ray_len)
    return dists

# =====================================================================
# EPISÓDIO DE EXECUÇÃO
# =====================================================================

def run_episode():
    """
    Roda um único episódio da simulação de navegação do aspirador.
    """
    global DIRT_IDS, DIRT_TOTAL_COUNT, OBST_DATA
    
    ensure_urdf()
    robot = p.loadURDF(URDF_NAME, basePosition=[0, 0, 0.2])
    traj = []
    t0 = time.time()
    
    
    
    jitter_cooldown = 0
    cooldown_phase = 0
    escape_turn = 0.0
    detour_active = False
    detour_point = None
    collision_to_target_count = 0
    home_active = False
    home_point = (0.0, 0.0)
    stuck_counter = 0
    last_pos = None
    orbit_counter = 0
    last_target_dist = None
    
    frame = 0 # Contador de frames para Node-RED
    
    while p.isConnected():
        pos, quat = p.getBasePositionAndOrientation(robot)
        
        roll, pitch, yaw = p.getEulerFromQuaternion(quat) 
        
        d = simple_lidar(robot)
        
        center = d[len(d)//2]
        left = d[0]; right = d[-1]
        
        vx = LINEAR_SPEED
        v_rot = 0.0
        
        target_bias = 0.0
        target_close = False
        
        current_status = "CLEANING"
        status_msg = "Procurando sujeira..."
        
        if ('DIRT_IDS' in globals()) and (not DIRT_IDS):
            home_active = True
            detour_active = False
            detour_point = None
            current_status = "HOME"
            status_msg = "Sujeira limpa. Voltando à base."

        if 'DIRT_IDS' in globals() and DIRT_IDS and not home_active:
            try:
                dirt_positions = [p.getBasePositionAndOrientation(di)[0] for di in DIRT_IDS]
                d2 = [math.hypot(dp[0]-pos[0], dp[1]-pos[1]) for dp in dirt_positions]
                idx = int(np.argmin(d2))
                tx, ty, _ = dirt_positions[idx]
                vx_t = tx - pos[0]
                vy_t = ty - pos[1]
                if detour_active and detour_point is not None:
                    tx, ty = detour_point[0], detour_point[1]
                    vx_t = tx - pos[0]
                    vy_t = ty - pos[1]
                    status_msg = "Desviando de rota."
                desired_yaw = math.atan2(vx_t, vy_t)
                ang_err = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi
                target_dist = math.hypot(vx_t, vy_t)
                if last_target_dist is not None:
                    improving = target_dist < last_target_dist - 0.01
                    near_walls = (
                        abs(tx - (-7.0)) < 0.5 or abs(7.0 - tx) < 0.5 or
                        abs(ty - (-3.0)) < 0.5 or abs(17.0 - ty) < 0.5
                    )
                    if near_walls and not improving and not detour_active:
                        orbit_counter += 1
                        current_status = "STUCK" # Detectando órbita ou preso
                        status_msg = "Possível órbita ou preso perto da parede."
                    else:
                        orbit_counter = max(0, orbit_counter - 1)
                last_target_dist = target_dist

                if orbit_counter > 20 and not detour_active:
                    normals = []
                    if abs(tx - (-7.0)) < 0.5: normals.append((1.0, 0.0))
                    if abs(7.0 - tx) < 0.5: normals.append((-1.0, 0.0))
                    if abs(ty - (-3.0)) < 0.5: normals.append((0.0, 1.0))
                    if abs(17.0 - ty) < 0.5: normals.append((0.0, -1.0))
                    nx = sum(n[0] for n in normals) if normals else 0.0
                    ny = sum(n[1] for n in normals) if normals else 0.0
                    offset = 0.8
                    wx = tx + nx * offset
                    wy = ty + ny * offset
                    margin = 0.25
                    wx = max(-7.0+margin, min(7.0-margin, wx))
                    wy = max(-3.0+margin, min(17.0-margin, wy))
                    detour_point = (wx, wy)
                    detour_active = True
                    orbit_counter = 0
                if not detour_active and d2[idx] < 0.25:
                    target_close = True
                    try:
                        p.removeBody(DIRT_IDS[idx])
                        DIRT_IDS.pop(idx)
                        status_msg = f"Sujeira consumida. Restam {len(DIRT_IDS)}"
                    except Exception:
                        pass
                    target_bias = 0.0
                else:
                    target_bias = np.clip(ang_err, -0.35, 0.35)
            except Exception:
                target_bias = 0.0
        elif home_active:
            tx, ty = home_point
            vx_t = tx - pos[0]
            vy_t = ty - pos[1]
            desired_yaw = math.atan2(vx_t, vy_t)
            ang_err = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi
            target_bias = np.clip(ang_err, -0.3, 0.3)
            if math.hypot(vx_t, vy_t) < 1.0:
                vx = min(vx, 0.4)

        dynamic_collision = COLLISION_DISTANCE * (0.5 if target_close else 1.0)
        if center < dynamic_collision:
            current_status = "COLLISION"
            status_msg = "Obstáculo à frente. Tentando desviar."
            try:
                start = [pos[0], pos[1], pos[2]+0.1]
                fwd = [pos[0] + math.sin(yaw)*0.8, pos[1] + math.cos(yaw)*0.8, start[2]]
                r = p.rayTest(start, fwd)[0]
                hit_normal = r[5] if len(r) > 5 else None
            except Exception:
                hit_normal = None
            if hit_normal:
                nx, ny, _ = hit_normal
                away_heading = math.atan2(-nx, -ny)
                ang_err = (away_heading - yaw + math.pi) % (2*math.pi) - math.pi
                v_rot = np.clip(ang_err, -0.6, 0.6)
                vx = 0.3
            else:
                if right > left:
                    v_rot = ROTATION_SPEED * 3.0
                else:
                    v_rot = -ROTATION_SPEED * 3.0
                vx = -0.25
        else:
            diff = left - right
            v_rot = -0.18 * diff + (0.5 if not target_close else 0.2) * target_bias
            if target_close:
                vx = min(vx, 0.3)
            if left < 0.25 and right < 0.25 and center < 0.35:
                current_status = "STUCK"
                status_msg = "Preso. Tentando girar no lugar."
                vx = 0.0
                v_rot = (ROTATION_SPEED * 4.0) * (1 if right > left else -1)
            try:
                if OBST_DATA:
                    near = sorted(OBST_DATA, key=lambda o: math.hypot(pos[0]-o[0], pos[1]-o[1]))[0]
                    dist = math.hypot(pos[0]-near[0], pos[1]-near[1])
                    if dist < 0.8:
                        away_heading = math.atan2(pos[0]-near[0], pos[1]-near[1])
                        ang_err = (away_heading - yaw + math.pi) % (2*math.pi) - math.pi
                        v_rot += np.clip(ang_err, -0.3, 0.3)
                        vx = min(vx, 0.5)
            except Exception:
                pass
            if not target_close and not home_active:
                wall_margin = 0.6
                d_left_wall = abs(pos[0] - (-7.0))
                d_right_wall = abs(7.0 - pos[0])
                d_bottom_wall = abs(pos[1] - (-3.0))
                d_top_wall = abs(17.0 - pos[1])
                near_wall = (d_left_wall < wall_margin or d_right_wall < wall_margin or d_bottom_wall < wall_margin or d_top_wall < wall_margin)
                if near_wall:
                    center_heading = math.atan2(0.0 - pos[0], 7.0 - pos[1])
                    ang_err = (center_heading - yaw + math.pi) % (2*math.pi) - math.pi
                    v_rot += np.clip(ang_err, -0.5, 0.5)
                    vx = min(vx, 0.5)

        if last_pos is not None:
            disp = math.hypot(pos[0]-last_pos[0], pos[1]-last_pos[1])
            if center < 0.4 and disp < 0.01:
                stuck_counter += 1
            else:
                stuck_counter = 0
        last_pos = pos
        if stuck_counter > 12:
            current_status = "STUCK"
            status_msg = "Preso. Executando manobra de resgate."
            vx = -0.3
            v_rot = (math.pi/2) * (1 if right > left else -1)
            stuck_counter = 0
        
        if jitter_cooldown > 0:
            current_status = "COLLISION"
            status_msg = "Manobra de escape ativa."
            cooldown_phase += 1
            if cooldown_phase < 8:
                vx = -0.22
                v_rot = escape_turn
            else:
                vx = 0.25
                v_rot = escape_turn + (math.pi/2) * (1 if right > left else -1)

        new_yaw = yaw + v_rot
        
        step_time = 0.04 
        dx = vx * math.sin(yaw) * step_time 
        dy = vx * math.cos(yaw) * step_time
        
        target_x = pos[0] + dx
        target_y = pos[1] + dy
        margin = 0.25
        target_x = max(-7.0+margin, min(7.0-margin, target_x))
        target_y = max(-3.0+margin, min(17.0-margin, target_y))
        
        new_quat = p.getQuaternionFromEuler([roll, pitch, new_yaw])
        
        p.resetBasePositionAndOrientation(robot, [target_x, target_y, pos[2]], new_quat)

        try:
            contacts = p.getContactPoints(bodyA=robot)
            contacts = [c for c in contacts if ('DIRT_IDS' not in globals()) or (c[2] not in DIRT_IDS and c[1] not in DIRT_IDS)]
            if contacts:
                p.addUserDebugLine([target_x, target_y, pos[2]+0.05], [target_x, target_y, pos[2]+0.35], [1,0,0], 3, lifeTime=0.2)
                if target_close:
                    collision_to_target_count += 1
                if jitter_cooldown == 0:
                    back_step = 0.15
                    target_x = pos[0] - back_step * math.sin(yaw)
                    target_y = pos[1] - back_step * math.cos(yaw)
                    try:
                        start = [pos[0], pos[1], pos[2]+0.1]
                        fwd = [pos[0] + math.sin(yaw)*0.6, pos[1] + math.cos(yaw)*0.6, start[2]]
                        r = p.rayTest(start, fwd)[0]
                        hit_normal = r[5] if len(r) > 5 else None
                    except Exception:
                        hit_normal = None
                    if hit_normal:
                        nx, ny, _ = hit_normal
                        away_heading = math.atan2(-nx, -ny)
                        escape_turn = (away_heading - yaw + math.pi) % (2*math.pi) - math.pi
                        escape_turn = np.clip(escape_turn, -0.8, 0.8)
                    else:
                        jitter = np.random.uniform(0.35, 0.6)
                        escape_turn = (jitter if right > left else -jitter)
                    new_yaw = yaw + escape_turn
                    new_quat = p.getQuaternionFromEuler([roll, pitch, new_yaw])
                    p.resetBasePositionAndOrientation(robot, [target_x, target_y, pos[2]], new_quat)
                    jitter_cooldown = 18
                    cooldown_phase = 0
                if collision_to_target_count > 6 and not detour_active:
                    try:
                        ax = math.cos(yaw)
                        ay = math.sin(yaw)
                        side = 1 if right > left else -1
                        wx = pos[0] + side * (-ay) * 0.8 + ax * 0.6
                        wy = pos[1] + side * (ax) * 0.8 + ay * 0.6
                        margin = 0.25
                        wx = max(-7.0+margin, min(7.0-margin, wx))
                        wy = max(-3.0+margin, min(17.0-margin, wy))
                        detour_point = (wx, wy)
                        detour_active = True
                        collision_to_target_count = 0
                    except Exception:
                        pass
        except Exception:
            pass

        try:
            p.resetDebugVisualizerCamera(cameraDistance=4.0,
                                         cameraYaw=0.0,
                                         cameraPitch=-45.0,
                                         cameraTargetPosition=[target_x, target_y, pos[2]])
        except Exception:
            pass

        traj.append((pos[0], pos[1]))
        
        # --- ENVIAR PARA NODE-RED (A cada 12 frames) ---
        if frame % 12 == 0:
            payload = {
                "x": float(pos[0]),
                "y": float(pos[1]),
                "dirt_remaining": len(DIRT_IDS),
                "dirt_total": DIRT_TOTAL_COUNT,
                "status": current_status,
                "status_msg": status_msg
            }
            send_to_node_red(payload)
        
        if len(traj) > MAX_STEPS: 
            current_status = "FINISHED"
            status_msg = "Limite de tempo atingido."
            send_to_node_red(payload) # Envia status final
            break
        
        try:
            p.stepSimulation()
        except Exception:
            break
        time.sleep(4./240.) 
        if jitter_cooldown > 0:
            jitter_cooldown -= 1
            if jitter_cooldown == 0:
                cooldown_phase = 0
        if detour_active and detour_point is not None:
            if math.hypot(pos[0]-detour_point[0], pos[1]-detour_point[1]) < 0.3:
                detour_active = False
                detour_point = None
        if home_active:
            if math.hypot(pos[0]-home_point[0], pos[1]-home_point[1]) < 0.25:
                current_status = "FINISHED"
                status_msg = "Limpeza e retorno concluídos!"
                send_to_node_red(payload) # Envia status final
                break
        
        frame += 1
        
    try:
        p.removeBody(robot)
    except Exception:
        pass
    return traj

# =====================================================================
# MAIN
# =====================================================================

def main():
    """
    Função principal que inicia o simulador e executa múltiplos episódios.
    """
    cid = p.connect(p.GUI)
    if cid < 0:
        cid = p.connect(p.DIRECT)
        if cid < 0:
            print("Falha ao conectar ao PyBullet.")
            return
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    p.resetSimulation()
    build_env()
    
    all_trajs = []
    
    for ep in range(3):
        if not p.isConnected():
            break
        traj = run_episode()
        all_trajs.append(traj)
        
        print(f"[Simulação] Episódio {ep+1} concluído. Duração (passos): {len(traj)}")
        
        time.sleep(0.5)
        
    print('aspirador done - Simulação concluída.')
    if p.isConnected():
        p.disconnect()

if __name__=='__main__': 
    main()
