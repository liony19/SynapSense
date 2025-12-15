import pybullet as p
import pybullet_data
import time, math, numpy as np, os, random, sys
import requests

NODE_RED_URL = "http://localhost:1880/dados-robo"
URDF_NAME = "mobile_robot_diff.urdf"

ROBOT_SIZE = 0.2
ARENA_RADIUS = 100.0 * ROBOT_SIZE

ROBOT_CLEARANCE = ROBOT_SIZE * 5.0

START_POS = (0.0, 0.0)

SPAWN_CLEARANCE = ROBOT_CLEARANCE
DIRT_SPAWN_INTERVAL = 2.0
DIRT_SPAWN_WINDOW = 120.0
CLEAN_DISTANCE = 1.2 * (ROBOT_SIZE * 0.5)
CLEAN_DISTANCE = 1.2 * (ROBOT_SIZE * 0.5)

VMAX = 1.0
SIM_DURATION = 180.0
DIRT_MAX = 60
RUN_DEADLINE = None

WHEEL_RADIUS = 0.07
WHEEL_CENTER_Z = -0.14

OBST_IDS = []
OBST_DATA = []
DIRT_IDS = []
DIRT_TOTAL_COUNT = 0
PLANE_ID = None

LINEAR_SPEED = 1.0 # Metade do tamnho do robô é lento demais, mas abaixo está essa versão se quiser testar
# LINEAR_SPEED = 0.5 * ROBOT_SIZE
COLLISION_DISTANCE = 1.4
ROTATION_SPEED = 0.18
MAX_STEPS = 3600
OBST_MIN_SEPARATION = ROBOT_SIZE * 4.0


def send_to_node_red(data_dict):
    try:
        requests.post(NODE_RED_URL, json=data_dict, timeout=0.05)
    except:
        pass


def ensure_urdf():
    if not os.path.exists(URDF_NAME):
        urdf = """<?xml version='1.0'?><robot name='mini'><link name='base'><visual><geometry><box size='0.4 0.3 0.1'/></geometry></visual><collision><geometry><box size='0.4 0.3 0.1'/></geometry></collision></link></robot>"""
        with open(URDF_NAME,'w') as f: f.write(urdf)


def build_env():
    global OBST_IDS, OBST_DATA, DIRT_IDS, DIRT_TOTAL_COUNT
    p.resetSimulation()
    global PLANE_ID
    PLANE_ID = p.loadURDF('plane.urdf')
    ring_posts = max(24, int(ARENA_RADIUS * 4))
    post_radius = ROBOT_SIZE * 0.08
    post_height = 1.0
    for i in range(ring_posts):
        a = 2 * math.pi * i / ring_posts
        px = math.cos(a) * ARENA_RADIUS
        py = math.sin(a) * ARENA_RADIUS
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=post_radius, height=post_height)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=post_radius, length=post_height, rgbaColor=[0.4, 0.4, 0.4, 1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=[px, py, post_height*0.5])

    OBST_IDS = []
    OBST_DATA = []
    arena_area = math.pi * (ARENA_RADIUS ** 2)
    target_obst_area = 0.01 * arena_area # Apenas 1% pois só 25% já ocupa espaço demais e demora muito pra carregar, abixo está a versão em que os obstáculos cobrem de 25% a 40%
    # target_obst_area = arena_area * random.uniform(0.25, 0.40)
    robot_area = math.pi * ((ROBOT_SIZE * 0.5) ** 2)
    min_obst_area = 0.5 * robot_area

    SECTORS = 32
    sector_area = [0.0] * SECTORS
    placed_area = 0.0
    max_attempts_per_sector = 120
    sector_idx = 0
    per_sector_target = target_obst_area / float(SECTORS)

    while placed_area < target_obst_area:
        attempts = 0
        placed_in_sector = False
        while attempts < max_attempts_per_sector and placed_area < target_obst_area:
            attempts += 1
            a0 = (2.0 * math.pi / SECTORS) * sector_idx
            a1 = a0 + (2.0 * math.pi / SECTORS)
            ang = random.uniform(a0, a1)
            r = random.uniform(SPAWN_CLEARANCE, ARENA_RADIUS - ROBOT_SIZE * 1.0)
            x = math.cos(ang) * r
            y = math.sin(ang) * r
            if math.hypot(x - START_POS[0], y - START_POS[1]) < SPAWN_CLEARANCE:
                continue

            shape_type = random.choice(['cyl', 'box'])
            if shape_type == 'cyl':
                min_rad = math.sqrt(min_obst_area / math.pi)
                max_rad = ROBOT_SIZE * 0.9
                col_rad = min_rad if max_rad <= min_rad else random.uniform(min_rad, max_rad)
                height = random.uniform(0.6, 1.2)
                this_area = math.pi * (col_rad ** 2)
            else:
                min_side = math.sqrt(min_obst_area)
                ax = random.uniform(min_side, max(ROBOT_SIZE * 1.0, min_side * 1.5))
                ay = random.uniform(min_side, max(ROBOT_SIZE * 1.0, min_side * 1.5))
                half_ext = [ax*0.5, ay*0.5, ROBOT_SIZE*0.3]
                this_area = ax * ay

            too_close = False
            approx_radius = math.sqrt(this_area / math.pi)
            for ox, oy, orad in OBST_DATA:
                safe_dist = approx_radius + orad + OBST_MIN_SEPARATION
                if math.hypot(x-ox, y-oy) < safe_dist:
                    too_close = True
                    break
            if too_close:
                continue

            if math.hypot(x - START_POS[0], y - START_POS[1]) < (SPAWN_CLEARANCE + approx_radius):
                continue

            if shape_type == 'cyl':
                col = p.createCollisionShape(p.GEOM_CYLINDER, radius=col_rad, height=height)
                vis = p.createVisualShape(p.GEOM_CYLINDER, radius=col_rad, length=height, rgbaColor=[0.3, 0.7, 0.3, 1])
                oid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=[x, y, height*0.5])
                OBST_IDS.append(oid)
                OBST_DATA.append((x, y, col_rad))
            else:
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_ext)
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_ext, rgbaColor=[0.2, 0.5, 0.7, 1])
                oid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=[x, y, half_ext[2]])
                OBST_IDS.append(oid)
                box_radius = math.hypot(half_ext[0], half_ext[1])
                OBST_DATA.append((x, y, box_radius))

            placed_area += this_area
            sector_area[sector_idx] += this_area
            placed_in_sector = True
            if sector_area[sector_idx] >= per_sector_target:
                break

        sector_idx = (sector_idx + 1) % SECTORS
        if not placed_in_sector:
            break

    DIRT_IDS = []
    DIRT_TOTAL_COUNT = 0


def simple_lidar(robot_id, n=9, ray_len=2.0):
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


def clamp_to_arena(x, y, margin=0.0):
    r = math.hypot(x, y)
    max_r = max(0.0, ARENA_RADIUS - margin)
    if r <= max_r or r == 0.0:
        return x, y
    fac = max_r / r
    return x * fac, y * fac


def run_episode():
    global DIRT_IDS, DIRT_TOTAL_COUNT, OBST_DATA
    ensure_urdf()
    try:
        base_z = WHEEL_RADIUS - WHEEL_CENTER_Z
    except Exception:
        base_z = 0.2
    robot = p.loadURDF(URDF_NAME, basePosition=[0, 0, float(base_z)])
    try:
        global PLANE_ID
        if PLANE_ID is not None:
            nj = p.getNumJoints(robot)
            try:
                p.setCollisionFilterPair(PLANE_ID, robot, -1, -1, 0)
            except Exception:
                pass
            for li in range(nj):
                try:
                    p.setCollisionFilterPair(PLANE_ID, robot, -1, li, 0)
                except Exception:
                    pass
    except Exception:
        pass

    traj = []
    t0 = time.time()
    last_spawn = -DIRT_SPAWN_INTERVAL
    collision_count = 0
    cleaned_times = []

    jitter_cooldown = 0
    cooldown_phase = 0
    escape_turn = 0.0
    detour_active = False
    detour_point = None
    detour_start_time = None
    detour_attempt_idx = 0
    collision_to_target_count = 0
    stuck_counter = 0
    last_pos = None
    orbit_counter = 0
    last_target_dist = None
    corridor_clear_to_target = False

    total_distance = 0.0
    last_clean_distance = 0.0
    cleaned_distances = []
    near_miss_count = 0
    near_miss_cooldown = 0
    energy_proxy_total = 0.0
    OCCUPANCY_GRID_SIZE = 24

    def _ensure_waypoint_clear(wx, wy):
        if not OBST_DATA:
            return clamp_to_arena(wx, wy, margin=0.25)
        for ox, oy, orad in OBST_DATA:
            d = math.hypot(wx - ox, wy - oy)
            min_clear = orad + ROBOT_SIZE * 0.9
            if d < min_clear:
                dx = wx - ox; dy = wy - oy
                nrm = math.hypot(dx, dy) or 1.0
                ux, uy = dx / nrm, dy / nrm
                wx = ox + ux * min_clear
                wy = oy + uy * min_clear
        return clamp_to_arena(wx, wy, margin=0.25)

    def _build_partial_maps(size=OCCUPANCY_GRID_SIZE):
        cell = (2.0 * ARENA_RADIUS) / float(size)
        occ = [0] * (size * size)
        dirt_map = [0] * (size * size)
        def idx_of(x, y):
            ix = int((x + ARENA_RADIUS) / cell)
            iy = int((y + ARENA_RADIUS) / cell)
            if ix < 0 or ix >= size or iy < 0 or iy >= size:
                return None
            return iy * size + ix
        for ox, oy, orad in OBST_DATA:
            i = idx_of(ox, oy)
            if i is not None:
                occ[i] = 1
        for did in DIRT_IDS:
            try:
                dpos = p.getBasePositionAndOrientation(did)[0]
                i = idx_of(dpos[0], dpos[1])
                if i is not None:
                    dirt_map[i] = 1
            except Exception:
                continue
        return occ, dirt_map

    frame = 0
    deadline_reached = False

    while p.isConnected():
        sim_time = time.time() - t0
        try:
            if RUN_DEADLINE is not None and time.time() >= RUN_DEADLINE:
                deadline_reached = True
                break
        except Exception:
            pass
        if sim_time >= SIM_DURATION:
            deadline_reached = True
            break
        if sim_time <= DIRT_SPAWN_WINDOW and (sim_time - last_spawn) >= DIRT_SPAWN_INTERVAL and DIRT_TOTAL_COUNT < DIRT_MAX:
            dirt_radius = 0.05
            attempts = 0
            placed = False
            margin_from_wall = max(0.6, ROBOT_SIZE * 3.0)
            max_attempts = 200
            while attempts < max_attempts and not placed:
                attempts += 1
                ang = random.uniform(0, 2 * math.pi)
                r = math.sqrt(random.uniform(0.0, 1.0)) * (ARENA_RADIUS - margin_from_wall)
                dx = math.cos(ang) * r
                dy = math.sin(ang) * r
                if math.hypot(dx - START_POS[0], dy - START_POS[1]) < SPAWN_CLEARANCE:
                    continue
                collides_obst = False
                for ox, oy, orad in OBST_DATA:
                    dco = math.hypot(dx - ox, dy - oy)
                    if dco < (orad + dirt_radius + max(0.06, OBST_MIN_SEPARATION * 0.5)):
                        collides_obst = True
                        break
                if collides_obst:
                    continue
                collides_dirt = False
                for did in DIRT_IDS:
                    try:
                        dpos = p.getBasePositionAndOrientation(did)[0]
                        if math.hypot(dx - dpos[0], dy - dpos[1]) < (dirt_radius * 2.0 + 0.05):
                            collides_dirt = True
                            break
                    except Exception:
                        continue
                if collides_dirt:
                    continue
                d_col = p.createCollisionShape(p.GEOM_SPHERE, radius=dirt_radius)
                d_vis = p.createVisualShape(p.GEOM_SPHERE, radius=dirt_radius, rgbaColor=[0.9, 0.85, 0.1, 1])
                did = p.createMultiBody(baseMass=0.01, baseCollisionShapeIndex=d_col, baseVisualShapeIndex=d_vis, basePosition=[dx, dy, dirt_radius])
                try:
                    p.setCollisionFilterGroupMask(did, -1, 0, 0)
                except Exception:
                    pass
                DIRT_IDS.append(did)
                DIRT_TOTAL_COUNT += 1
                placed = True
            last_spawn = sim_time
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
            current_status = "IDLE"
            status_msg = "Sem sujeira — explorando (aguardando novos spawns)."
            vx = min(vx, LINEAR_SPEED * 0.75)
            if frame % 60 == 0:
                v_rot = v_rot + random.uniform(-0.12, 0.12)
            else:
                v_rot = 0.0

        if 'DIRT_IDS' in globals() and DIRT_IDS:
            try:
                to_remove = []
                for i, did in enumerate(list(DIRT_IDS)):
                    try:
                        dpos = p.getBasePositionAndOrientation(did)[0]
                    except Exception:
                        to_remove.append(did)
                        continue
                    for ox, oy, orad in OBST_DATA:
                        if math.hypot(dpos[0]-ox, dpos[1]-oy) < (orad + 0.06 + 0.02):
                            try:
                                p.removeBody(did)
                            except Exception:
                                pass
                            to_remove.append(did)
                            break
                for did in to_remove:
                    try:
                        DIRT_IDS.remove(did)
                    except Exception:
                        pass
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
                    near_walls = math.hypot(tx, ty) > (ARENA_RADIUS - 0.5)
                    if near_walls and not improving and not detour_active:
                        orbit_counter += 1
                        current_status = "STUCK"
                        status_msg = "Possível órbita ou preso perto da borda da arena."
                    else:
                        orbit_counter = max(0, orbit_counter - 1)
                last_target_dist = target_dist

                if orbit_counter > 20 and not detour_active:
                        txn, tyn = tx, ty
                        d = math.hypot(txn, tyn)
                        if d == 0:
                            nx, ny = 0.0, 0.0
                        else:
                            nx, ny = -txn / d, -tyn / d
                        offset = 0.8
                        wx = tx + nx * offset
                        wy = ty + ny * offset
                        wx, wy = clamp_to_arena(wx, wy, margin=0.25)
                        detour_point = (wx, wy)
                        detour_active = True
                        detour_start_time = sim_time
                        detour_attempt_idx = 0
                        orbit_counter = 0
                if not detour_active and d2[idx] < CLEAN_DISTANCE:
                    target_close = True
                    try:
                        p.removeBody(DIRT_IDS[idx])
                        DIRT_IDS.pop(idx)
                        status_msg = f"Sujeira consumida. Restam {len(DIRT_IDS)}"
                        try:
                            cleaned_times.append(sim_time)
                            try:
                                dist_since_last = total_distance - last_clean_distance
                                cleaned_distances.append(dist_since_last)
                                last_clean_distance = total_distance
                            except Exception:
                                pass
                        except Exception:
                            pass
                    except Exception:
                        pass
                    target_bias = 0.0
                else:
                    target_bias = np.clip(ang_err, -0.35, 0.35)
            except Exception:
                target_bias = 0.0

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
                v_rot = np.clip(ang_err, -0.4, 0.4)
                vx = 0.2
            else:
                if right > left:
                    v_rot = ROTATION_SPEED * 3.0
                else:
                    v_rot = -ROTATION_SPEED * 3.0
                vx = -0.12
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
                    repulsion_radius = 0.38
                    rep_x = 0.0
                    rep_y = 0.0
                    nearest = None
                    nearest_d = 1e9

                    forward_dist = 1.0
                    forward_unit = (math.sin(yaw), math.cos(yaw))
                    lateral_unit = (math.cos(yaw), -math.sin(yaw))

                    for ox, oy, orad in OBST_DATA:
                        dxo = pos[0] - ox
                        dyo = pos[1] - oy
                        d_center = math.hypot(dxo, dyo)
                        d_surface = d_center - orad
                        if d_surface < nearest_d:
                            nearest_d = d_surface
                            nearest = (ox, oy, orad)

                        relx = ox - pos[0]
                        rely = oy - pos[1]
                        fwd = relx * forward_unit[0] + rely * forward_unit[1]
                        lat = relx * lateral_unit[0] + rely * lateral_unit[1]

                        if ('corridor_clear_to_target' in locals()) and corridor_clear_to_target and (0.0 < fwd <= forward_dist) and abs(lat) < (ROBOT_SIZE * 1.05 * 0.6):
                            continue

                        if d_surface < repulsion_radius:
                            w = max(0.0, (repulsion_radius - d_surface) / repulsion_radius)
                            if d_center <= 1e-6:
                                ux = random.uniform(-1.0, 1.0)
                                uy = random.uniform(-1.0, 1.0)
                                nrm = math.hypot(ux, uy) or 1.0
                                ux /= nrm; uy /= nrm
                            else:
                                ux = dxo / d_center
                                uy = dyo / d_center
                            rep_x += ux * w
                            rep_y += uy * w

                    rep_strength = math.hypot(rep_x, rep_y)
                    if rep_strength > 1e-6:
                        away_ang = math.atan2(rep_x, rep_y)
                        ang_err_rep = (away_ang - yaw + math.pi) % (2*math.pi) - math.pi
                        rep_scale = 0.6 if not target_close else 0.22
                        v_rot += np.clip(ang_err_rep * rep_scale, -0.9, 0.9)
                        if rep_strength > 0.25:
                            vx = min(vx, max(0.05, VMAX * (1.0 - rep_strength)))
                        if rep_strength > 1.1:
                            jitter_cooldown = 18
                            cooldown_phase = 0
                            escape_turn = np.clip(ang_err_rep, -0.8, 0.8)

                    if nearest is not None and nearest_d < 0.6 and target_close:
                        vx = min(vx, 0.42)
            except Exception:
                pass
            try:
                est_step = 0.04
                energy_proxy_total += abs(vx) * abs(v_rot) * est_step
            except Exception:
                pass
            try:
                if near_miss_cooldown > 0:
                    near_miss_cooldown -= 1
                near_threshold = dynamic_collision * 1.2
                if center < near_threshold and center >= dynamic_collision and near_miss_cooldown == 0:
                    near_miss_count += 1
                    near_miss_cooldown = 12
            except Exception:
                pass
            if detour_active and detour_start_time is not None and (sim_time - detour_start_time) >= 5.0:
                try:
                    if ('tx' in locals()):
                        def path_blocked_count_local(ax, ay, bx, by):
                            vecx = bx - ax
                            vecy = by - ay
                            dist = math.hypot(vecx, vecy)
                            if dist < 0.001:
                                return 0
                            steps = max(3, int(dist / 0.25))
                            halfw = ROBOT_SIZE * 0.55
                            blocked = 0
                            for si in range(1, steps + 1):
                                frac = si / float(steps)
                                fx = ax + vecx * frac
                                fy = ay + vecy * frac
                                for ox, oy, orad in OBST_DATA:
                                    if math.hypot(fx - ox, fy - oy) - orad < halfw:
                                        blocked += 1
                                        break
                            return blocked

                        current_blocked = path_blocked_count_local(pos[0], pos[1], tx, ty)
                        offsets = [-math.pi/3, -math.pi/6, math.pi/6, math.pi/3]
                        best = None
                        best_score = current_blocked
                        desired_yaw = math.atan2(tx - pos[0], ty - pos[1])
                        for i, off in enumerate(offsets):
                            if i == detour_attempt_idx:
                                continue
                            ayaw = desired_yaw + off
                            wdist = min(max(0.6, math.hypot(tx-pos[0], ty-pos[1]) * 0.5), 2.5)
                            wx = pos[0] + math.sin(ayaw) * wdist
                            wy = pos[1] + math.cos(ayaw) * wdist
                            score = path_blocked_count_local(pos[0], pos[1], wx, wy) + path_blocked_count_local(wx, wy, tx, ty)
                            if score < best_score:
                                best_score = score
                                best = (wx, wy, i)
                        if best is not None:
                            wx, wy, bi = best
                            wx, wy = _ensure_waypoint_clear(wx, wy)
                            detour_point = (wx, wy)
                            detour_active = True
                            detour_start_time = sim_time
                            detour_attempt_idx = bi
                        else:
                            detour_active = False
                            detour_point = None
                except Exception:
                    pass
            if not target_close:
                wall_margin = 0.6
                d_to_center = math.hypot(pos[0], pos[1])
                near_wall = d_to_center > (ARENA_RADIUS - wall_margin)
                if near_wall:
                    center_heading = math.atan2(-pos[0], -pos[1])
                    ang_err = (center_heading - yaw + math.pi) % (2*math.pi) - math.pi
                    v_rot += np.clip(ang_err, -0.5, 0.5)
                    vx = min(vx, 0.5)

        if last_pos is not None:
            disp = math.hypot(pos[0]-last_pos[0], pos[1]-last_pos[1])
            try:
                total_distance += disp
            except Exception:
                pass
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
                vx = -0.15
                v_rot = escape_turn * 0.9
            else:
                vx = 0.18
                v_rot = escape_turn + (math.pi/3) * (1 if right > left else -1)

        if sim_time <= DIRT_SPAWN_WINDOW:
            try:
                vx = max(-VMAX, min(VMAX, vx))
            except Exception:
                vx = max(-0.5 * ROBOT_SIZE, min(0.5 * ROBOT_SIZE, vx))

        new_yaw = yaw + v_rot
        step_time = 0.04 
        dx = vx * math.sin(yaw) * step_time 
        dy = vx * math.cos(yaw) * step_time
        target_x = pos[0] + dx
        target_y = pos[1] + dy
        margin = 0.25
        target_x, target_y = clamp_to_arena(target_x, target_y, margin=margin)
        new_quat = p.getQuaternionFromEuler([roll, pitch, new_yaw])
        if abs(target_x) < 1e-6 and abs(target_y) < 1e-6 and math.hypot(pos[0], pos[1]) > 0.05:
            try:
                p.addUserDebugText('Suspicious teleport blocked', [pos[0], pos[1], pos[2]+0.3], [1,0.5,0], textSize=1.2, lifeTime=0.5)
            except Exception:
                pass
            target_x, target_y = pos[0], pos[1]

        p.resetBasePositionAndOrientation(robot, [target_x, target_y, pos[2]], new_quat)

        try:
            contacts = p.getContactPoints(bodyA=robot)
            contacts = [c for c in contacts if ('DIRT_IDS' not in globals()) or (c[2] not in DIRT_IDS and c[1] not in DIRT_IDS)]
            if contacts:
                collision_count += 1
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
                        wx, wy = clamp_to_arena(wx, wy, margin=0.25)
                        wx, wy = _ensure_waypoint_clear(wx, wy)
                        detour_point = (wx, wy)
                        detour_active = True
                        detour_start_time = sim_time
                        detour_attempt_idx = 0
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
        if frame % 12 == 0:
            if OBST_DATA:
                nod = min(max(0.0, math.hypot(pos[0]-ox, pos[1]-oy) - orad) for ox, oy, orad in OBST_DATA)
            else:
                nod = None
            try:
                occ_map, dirt_map = _build_partial_maps()
            except Exception:
                occ_map, dirt_map = [], []
            dirt_positions_list = []
            for did in DIRT_IDS:
                try:
                    dpos = p.getBasePositionAndOrientation(did)[0]
                    dirt_positions_list.append([float(dpos[0]), float(dpos[1])])
                except Exception:
                    continue
            try:
                if len(cleaned_times) >= 2:
                    cleaned_intervals = [round(cleaned_times[i] - cleaned_times[i-1], 3) for i in range(1, len(cleaned_times))]
                else:
                    cleaned_intervals = []
            except Exception:
                cleaned_intervals = []
            payload = {
                "t": round(sim_time, 3),
                "x": float(pos[0]),
                "y": float(pos[1]),
                "heading": float(yaw),
                "v_linear": float(vx),
                "v_rot": float(v_rot),
                "nearest_obstacle_dist": nod,
                "dirt_remaining": len(DIRT_IDS),
                "dirt_generated": DIRT_TOTAL_COUNT,
                "dirt_cleaned": max(0, DIRT_TOTAL_COUNT - len(DIRT_IDS)),
                "dirt_positions": dirt_positions_list,
                "cleaned_times": cleaned_times,
                "cleaned_intervals": cleaned_intervals,
                "cleaned_distances": cleaned_distances,
                "total_distance": total_distance,
                "occupancy_map": occ_map,
                "dirt_map": dirt_map,
                "near_misses": near_miss_count,
                "energy_proxy_total": energy_proxy_total,
                "status": current_status,
                "status_msg": status_msg,
                "collisions": collision_count
            }
            send_to_node_red(payload)
        if len(traj) > MAX_STEPS: 
            current_status = "FINISHED"
            status_msg = "Limite de tempo atingido."
            send_to_node_red(payload)
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
        frame += 1
    try:
        p.removeBody(robot)
    except Exception:
        pass
    if deadline_reached:
        try:
            send_to_node_red({"summary": {"note": "deadline_reached"}})
        except Exception:
            pass
    total_generated = DIRT_TOTAL_COUNT
    total_cleaned = max(0, DIRT_TOTAL_COUNT - len(DIRT_IDS))
    pct_cleaned = (total_cleaned / total_generated * 100.0) if total_generated else 0.0
    if len(cleaned_times) >= 2:
        diffs = [cleaned_times[i] - cleaned_times[i-1] for i in range(1, len(cleaned_times))]
        mean_time = sum(diffs) / len(diffs)
    else:
        mean_time = None
    summary = {
        "total_generated": total_generated,
        "total_cleaned": total_cleaned,
        "percent_cleaned": pct_cleaned,
        "mean_time_between_clean": mean_time,
        "efficiency_time_mean": mean_time,
        "efficiency_distance_per_clean": (total_distance / total_cleaned) if total_cleaned else None,
        "collisions": collision_count,
        "near_misses": near_miss_count,
        "energy_proxy_total": energy_proxy_total,
        "cleaned_distances": cleaned_distances
    }
    send_to_node_red({"summary": summary})
    return traj


def main():
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
    run_start = time.time()
    for ep in range(3):
        if not p.isConnected():
            break
        try:
            global RUN_DEADLINE
            if RUN_DEADLINE is None:
                RUN_DEADLINE = run_start + SIM_DURATION
        except Exception:
            pass
        traj = run_episode()
        all_trajs.append(traj)
        print(f"[Simulação] Episódio {ep+1} concluído. Duração (passos): {len(traj)}")
        try:
            if RUN_DEADLINE is not None and time.time() >= RUN_DEADLINE:
                try:
                    p.disconnect()
                except Exception:
                    pass
                try:
                    os._exit(0)
                except Exception:
                    sys.exit(0)
        except Exception:
            pass
        time.sleep(0.5)
    print('aspirador done - Simulação concluída.')
    if p.isConnected():
        p.disconnect()

if __name__=='__main__': 
    main()
