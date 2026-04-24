#!/usr/bin/env python3
import traci
import random
import time
import os
import json
from collections import deque
from pathlib import Path

# =========================================================
# CARGAR CONFIGURACIÓN DESDE VARIABLES DE ENTORNO
# =========================================================
_CONFIG_MODE = os.getenv('AMB_SIM_MODE', 'DEFAULT')
_USE_GUI = os.getenv('AMB_USE_GUI', 'true').lower() == 'true'
_ENABLE_TLS = os.getenv('AMB_ENABLE_TLS_CONTROL', 'true').lower() == 'true'
_ENABLE_COOP = os.getenv('AMB_ENABLE_VEHICLE_COOPERATION', 'true').lower() == 'true'
_ENABLE_REROUTE = os.getenv('AMB_ENABLE_GLOBAL_REROUTE', 'false').lower() == 'true'
_ENABLE_TELEPORT = os.getenv('AMB_ENABLE_TELEPORT', 'false').lower() == 'true' # AHORA SÍ SE RESPETA
_OUTPUT_METRICS = os.getenv('AMB_OUTPUT_METRICS', 'false').lower() == 'true'
_METRICS_FILE = os.getenv('AMB_METRICS_FILE', 'metrics.json')
_SEED_OVERRIDE = os.getenv('AMB_SEED', None)

# =========================================================
# CONFIGURACIÓN GENERAL
# =========================================================
SUMO_CFG = "/root/omnet/taller3/map5.sumocfg"
SUMO_BINARY = "sumo-gui" if _USE_GUI else "sumo"  
SIM_DURATION = 3600

# 1. Aumentar detección de semáforos (mirar más lejos y antes)
TLS_PREEMPTION_DISTANCE = 140.0          
DESTINATION_TLS_PREEMPTION_DISTANCE = 220.0 
TLS_SCAN_AHEAD = 4                       

# 2. Reducir la eliminación de vehículos atascados (menos "teletransportes")
AMBULANCE_WAIT_BEFORE_BLOCKER_REMOVE = 35.0 
TELEPORT_WAIT_THRESHOLD = 50.0           
MAX_TELEPORTS_PER_CALL = 1               

# Debug / reproducibilidad
DEBUG = True
USE_TIME_SEED = _SEED_OVERRIDE is None
RANDOM_SEED = int(_SEED_OVERRIDE) if _SEED_OVERRIDE else (int(time.time()) if USE_TIME_SEED else 42)
random.seed(RANDOM_SEED)

# Accidente
ACCIDENT_TIME = random.randint(800, 2000)
ACCIDENT_DURATION = 360.0
ACCIDENT_CLEAR_DELAY = 15.0
ALERT_INTERVAL = 30
ACCIDENT_TLS_NEAR_DISTANCE = 80.0
ACCIDENT_MIN_SPEED = 2.0
ACCIDENT_MIN_POS_FROM_START = 20.0
ACCIDENT_MIN_POS_FROM_END = 25.0
ACCIDENT_MIN_DISTANCE_FROM_BASE = 250.0
MIN_ACC_ROUTE_LENGTH_FROM_BASE = 8

# Ambulancia
AMBULANCE_ID = "ambulancia_0"
AMBULANCE_TYPE_ID = "ambulancia"
AMBULANCE_DELAY = 5
AMBULANCE_BASE_EDGE_ID = "-241308383"   
STRICT_AMBULANCE_BASE = False
MIN_ROUTE_LENGTH_FOR_AMBULANCE = 2
AMBULANCE_MAX_SPEED = 16.0
AMBULANCE_STRONG_MAX_SPEED = 18.0

# Selección automática de base de ambulancia (secundaria)
LEFT_PERCENTILE = 0.15
BOTTOM_PERCENTILE = 0.20
MAX_BASE_CANDIDATES = 12

# Prioridad semafórica
TLS_HOLD_STEPS = 15
TLS_STOP_DIST = 24.0
TLS_STOP_DIST_STRONG = 40.0
TLS_MIN_REAPPLY_GAP = 8

# Intersecciones "partidas" / box guard
SPLIT_TLS_NEIGHBOR_DISTANCE = 45.0
ENABLE_NETWORK_BOX_GUARD = False
ENABLE_BOX_GUARD_BEFORE_ACCIDENT = False
BOX_GUARD_START_DELAY_AFTER_ACCIDENT = 15
BOX_GUARD_INTERVAL = 6
BOX_GUARD_MIN_FREE_OUT_LANE = 22.0
BOX_GUARD_OUT_HALTING_THRESHOLD = 2
BOX_GUARD_ENTRY_DISTANCE = 12.0
BOX_GUARD_ACCEL = 4.0

# Cooperación vehicular / corredor
CLEARING_DISTANCE_AHEAD = 80.0
CLEARING_DISTANCE_BEHIND = 15.0
SLOWDOWN_SPEED = 1.5
ACCIDENT_ZONE_DISTANCE = 120.0
CORRIDOR_EDGES_AHEAD = 3

# Deadlock / recuperación gradual
AMB_HISTORY_LEN = 12
STUCK_MIN_PROGRESS = 5.0
STUCK_MAX_AVG_SPEED = 0.5
STUCK_RESOLVE_COOLDOWN = 10
BLOCKER_REMOVE_DISTANCE = 30.0
EMERGENCY_MOVE_DISTANCE = 10.0
AMBULANCE_WAIT_BEFORE_EMERGENCY_MOVE = 24.0
SINGLE_LANE_RELIEF_DISTANCE = 150.0
EMERGENCY_MOVE_MIN_DIST_TO_TLS = 20.0
EMERGENCY_MOVE_MIN_DIST_TO_LANE_END = 20.0
ENABLE_EMERGENCY_MOVETO = False   

# Gestión global de tráfico
ENABLE_GLOBAL_REROUTE_BEFORE_ACCIDENT = True
GLOBAL_REROUTE_START_DELAY_AFTER_ACCIDENT = 30 
TRAFFIC_MONITOR_INTERVAL = 120
CONGESTION_HALTING_THRESHOLD = 12
REROUTE_PROBABILITY = 0.10
REROUTE_EXCLUSION_EDGES_AHEAD = 4
ACCIDENT_NEIGHBORHOOD_EXCLUSION = 2
MIN_DIST_TO_LANE_END_FOR_REROUTE = 25.0
MAX_REROUTES_PER_CYCLE = 3
REROUTE_MIN_SPEED = 1.0

# Llegada
AMBULANCE_TARGET_OFFSET = 12.0
ARRIVAL_TOLERANCE = 10.0

# =========================================================
# ESTADO GLOBAL Y MÁQUINAS DE ESTADOS
# =========================================================
accidente_generado = False
vehiculo_accidentado = None
edge_accidente = None
lane_accidente = None
pos_accidente = None
ambulance_target_pos = None
accident_time_real = None
accident_near_tls = False
accident_tls_ids = []
accident_route_from_base = []

ambulancia_creada = False
ambulancia_despachada = False
ambulancia_llego = False
ambulancia_arrival_time = None
accidente_liberado = False
ambulancia_edge_origen = None
ambulance_base_candidates = []
last_route_id = None
ambulance_dispatch_failed = False

# Historial y recuperación
ambulance_history = deque(maxlen=AMB_HISTORY_LEN)
last_stuck_resolution_step = -999999
ambulance_recovery_level = 0
total_tls_interventions = 0 # Agregado para métricas precisas

# Control de semáforos
tls_priority_memory = {}
tls_original_program = {}
tls_cooldown = {}

# Logs
event_log = []

class TrafficState:
    NORMAL = "NORMAL"
    CONGESTION_DETECTED = "CONGESTION_DETECTED"
    REROUTING = "REROUTING"

global_traffic_state = TrafficState.NORMAL
congested_edges_memory = set()

class AmbulanceState:
    IDLE = "IDLE"
    DISPATCHED = "DISPATCHED"
    EN_ROUTE = "EN_ROUTE"
    PRECLEARING_TLS = "PRECLEARING_TLS"
    STUCK_L1 = "STUCK_L1"
    STUCK_L2 = "STUCK_L2"
    STUCK_L3 = "STUCK_L3"
    ON_SCENE = "ON_SCENE"
    CLEARED = "CLEARED"

ambulance_state = AmbulanceState.IDLE

# =========================================================
# UTILIDADES BÁSICAS Y SEGURIDAD SUMO
# =========================================================
def log_event(message: str):
    print(message)
    event_log.append(message)

def safe(func, default=None):
    try:
        return func()
    except traci.TraCIException:
        return default

def safe_get_vehicle_class(veh_id): return safe(lambda: traci.vehicle.getVehicleClass(veh_id))
def safe_get_vehicle_road(veh_id): return safe(lambda: traci.vehicle.getRoadID(veh_id))
def safe_get_vehicle_lane_id(veh_id): return safe(lambda: traci.vehicle.getLaneID(veh_id))
def safe_get_vehicle_lane_index(veh_id): return safe(lambda: traci.vehicle.getLaneIndex(veh_id))
def safe_get_vehicle_lane_pos(veh_id): return safe(lambda: traci.vehicle.getLanePosition(veh_id))
def safe_get_vehicle_speed(veh_id): return safe(lambda: traci.vehicle.getSpeed(veh_id), 0.0)
def safe_get_vehicle_length(veh_id): return safe(lambda: traci.vehicle.getLength(veh_id), 5.0)
def safe_get_vehicle_waiting_time(veh_id): return safe(lambda: traci.vehicle.getWaitingTime(veh_id), 0.0)
def safe_get_lane_length(lane_id): return safe(lambda: traci.lane.getLength(lane_id))
def safe_get_lane_shape(lane_id): return safe(lambda: traci.lane.getShape(lane_id), [])
def safe_get_edge_lane_number(edge_id): return safe(lambda: traci.edge.getLaneNumber(edge_id), 0)
def safe_get_vehicle_route_index(veh_id): return safe(lambda: traci.vehicle.getRouteIndex(veh_id), -1)
def safe_get_vehicle_allowed_speed(veh_id): return safe(lambda: traci.vehicle.getAllowedSpeed(veh_id), 13.9)
def safe_get_edge_halting(edge_id): return safe(lambda: traci.edge.getLastStepHaltingNumber(edge_id), 0)
def safe_get_edge_vehicle_ids(edge_id): return safe(lambda: traci.edge.getLastStepVehicleIDs(edge_id), [])
def safe_get_lane_vehicle_ids(lane_id): return safe(lambda: traci.lane.getLastStepVehicleIDs(lane_id), [])
def safe_get_lane_halting(lane_id): return safe(lambda: traci.lane.getLastStepHaltingNumber(lane_id), 0)

def safe_get_vehicle_route(veh_id):
    route = safe(lambda: traci.vehicle.getRoute(veh_id), [])
    return list(route) if route else []

def is_valid_normal_edge(edge_id: str) -> bool:
    return bool(edge_id) and not edge_id.startswith(":")

def is_internal_edge(edge_id: str) -> bool:
    return bool(edge_id) and edge_id.startswith(":")

def is_internal_lane(lane_id: str) -> bool:
    return bool(lane_id) and lane_id.startswith(":")

def lane_id_from_edge_and_index(edge_id: str, lane_idx: int) -> str:
    return f"{edge_id}_{lane_idx}"

def get_edge_centroid(edge_id: str):
    shape = safe_get_lane_shape(lane_id_from_edge_and_index(edge_id, 0))
    if not shape:
        return None
    return sum(p[0] for p in shape) / len(shape), sum(p[1] for p in shape) / len(shape)

def get_candidate_start_edges():
    return [e for e in traci.edge.getIDList() if is_valid_normal_edge(e) and safe_get_edge_lane_number(e) > 0]

def euclidean_distance(a, b):
    if a is None or b is None:
        return float("inf")
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

def debug(message: str):
    if DEBUG:
        log_event(message)

def safe_remove_vehicle(veh_id: str):
    try:
        if veh_id in traci.vehicle.getIDList():
            traci.vehicle.remove(veh_id)
    except Exception:
        pass

def ambulance_is_in_internal():
    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return False
    amb_edge = safe_get_vehicle_road(AMBULANCE_ID)
    amb_lane = safe_get_vehicle_lane_id(AMBULANCE_ID)
    return is_internal_edge(amb_edge) or is_internal_lane(amb_lane)

def get_remaining_route_edges(veh_id: str):
    route = safe_get_vehicle_route(veh_id)
    idx = safe_get_vehicle_route_index(veh_id)
    if not route or idx < 0 or idx >= len(route):
        return []
    return list(route[idx:])

def get_ambulance_protected_corridor():
    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return set()
    route = safe_get_vehicle_route(AMBULANCE_ID)
    idx = safe_get_vehicle_route_index(AMBULANCE_ID)
    if not route or idx < 0:
        return set()
    return set(route[idx: idx + REROUTE_EXCLUSION_EDGES_AHEAD])

def get_accident_exclusion_edges():
    exclusion = set()
    if edge_accidente:
        exclusion.add(edge_accidente)
    if AMBULANCE_ID in traci.vehicle.getIDList():
        route = safe_get_vehicle_route(AMBULANCE_ID)
        idx = safe_get_vehicle_route_index(AMBULANCE_ID)
        if route and idx >= 0:
            for e in route[max(0, idx - ACCIDENT_NEIGHBORHOOD_EXCLUSION): idx + ACCIDENT_NEIGHBORHOOD_EXCLUSION + 1]:
                exclusion.add(e)
    return exclusion

def get_protected_corridor_edges(veh_id: str, n_edges: int = CORRIDOR_EDGES_AHEAD):
    remaining = list(get_remaining_route_edges(veh_id))
    return remaining[:max(1, n_edges)] if remaining else []

def is_edge_in_protected_corridor(edge_id: str):
    if not ambulancia_creada or AMBULANCE_ID not in traci.vehicle.getIDList():
        return False
    return edge_id in get_protected_corridor_edges(AMBULANCE_ID)

def distance_to_end_of_lane(veh_id: str):
    lane_id = safe_get_vehicle_lane_id(veh_id)
    pos = safe_get_vehicle_lane_pos(veh_id)
    if lane_id is None or pos is None:
        return None
    lane_len = safe_get_lane_length(lane_id)
    if lane_len is None:
        return None
    return max(0.0, lane_len - pos)

def reroute_ambulance_if_needed():
    return False

def boost_ambulance_speed(ambulance_id: str, strong: bool = False):
    try:
        allowed = safe_get_vehicle_allowed_speed(ambulance_id)
        current = safe_get_vehicle_speed(ambulance_id)
        target = min(allowed, AMBULANCE_STRONG_MAX_SPEED if strong else AMBULANCE_MAX_SPEED)
        if target > current:
            traci.vehicle.slowDown(ambulance_id, target, 1)
    except traci.TraCIException:
        pass

def is_vehicle_safe_to_reroute(veh_id: str) -> bool:
    road_id = safe_get_vehicle_road(veh_id)
    lane_id = safe_get_vehicle_lane_id(veh_id)
    speed = safe_get_vehicle_speed(veh_id)

    if road_id is None or lane_id is None:
        return False
    if is_internal_edge(road_id) or is_internal_lane(lane_id):
        return False
    if speed is None or speed < REROUTE_MIN_SPEED:
        return False
    dist_end = distance_to_end_of_lane(veh_id)
    if dist_end is None or dist_end < MIN_DIST_TO_LANE_END_FOR_REROUTE:
        return False
    route = safe_get_vehicle_route(veh_id)
    idx = safe_get_vehicle_route_index(veh_id)
    if not route or idx < 0:
        return False
    return True

def ambulance_is_too_close_to_junction():
    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return True
    if ambulance_is_in_internal():
        return True

    dist_end = distance_to_end_of_lane(AMBULANCE_ID)
    if dist_end is None or dist_end < EMERGENCY_MOVE_MIN_DIST_TO_LANE_END:
        return True

    next_tls = safe(lambda: traci.vehicle.getNextTLS(AMBULANCE_ID), [])
    if next_tls:
        try:
            dist_tls = float(next_tls[0][2])
            if dist_tls < EMERGENCY_MOVE_MIN_DIST_TO_TLS:
                return True
        except Exception:
            pass
    return False

# =========================================================
# BASE FIJA DE AMBULANCIA
# =========================================================
def compute_ambulance_base_candidates():
    edge_positions = []
    for edge_id in traci.edge.getIDList():
        if not is_valid_normal_edge(edge_id) or safe_get_edge_lane_number(edge_id) < 1:
            continue
        centroid = get_edge_centroid(edge_id)
        if centroid is None:
            continue
        edge_positions.append((edge_id, centroid[0], centroid[1]))

    if not edge_positions:
        return []

    edge_positions.sort(key=lambda item: item[1])
    left_slice = edge_positions[:max(1, int(len(edge_positions) * LEFT_PERCENTILE))]
    left_slice.sort(key=lambda item: item[2])
    bottom_left = left_slice[:max(1, int(len(left_slice) * max(BOTTOM_PERCENTILE, 0.3)))]
    return [edge_id for edge_id, _, _ in bottom_left[:MAX_BASE_CANDIDATES]]

def get_primary_base_edge():
    global ambulance_base_candidates
    if AMBULANCE_BASE_EDGE_ID is not None:
        return AMBULANCE_BASE_EDGE_ID
    return ambulance_base_candidates[0] if ambulance_base_candidates else None

def choose_fixed_ambulance_origin(accident_edge: str):
    global ambulance_base_candidates

    if AMBULANCE_BASE_EDGE_ID is not None:
        try:
            route = traci.simulation.findRoute(AMBULANCE_BASE_EDGE_ID, accident_edge, vType=AMBULANCE_TYPE_ID)
            if route and len(route.edges) >= MIN_ROUTE_LENGTH_FOR_AMBULANCE and safe_get_edge_lane_number(AMBULANCE_BASE_EDGE_ID) > 0:
                return AMBULANCE_BASE_EDGE_ID, list(route.edges)
            if STRICT_AMBULANCE_BASE:
                return None, None
        except traci.TraCIException as e:
            if STRICT_AMBULANCE_BASE:
                return None, None

    for origin in ambulance_base_candidates:
        if origin == accident_edge:
            continue
        try:
            route = traci.simulation.findRoute(origin, accident_edge, vType=AMBULANCE_TYPE_ID)
            if route and len(route.edges) >= MIN_ROUTE_LENGTH_FOR_AMBULANCE:
                return origin, list(route.edges)
        except traci.TraCIException:
            continue

    for origin in get_candidate_start_edges():
        if origin == accident_edge:
            continue
        try:
            route = traci.simulation.findRoute(origin, accident_edge, vType=AMBULANCE_TYPE_ID)
            if route and len(route.edges) >= MIN_ROUTE_LENGTH_FOR_AMBULANCE:
                return origin, list(route.edges)
        except traci.TraCIException:
            continue

    return None, None

def get_tls_centroid(tls_id: str):
    lanes = safe(lambda: traci.trafficlight.getControlledLanes(tls_id), [])
    pts = []
    for lane_id in lanes:
        if not lane_id:
            continue
        shape = safe_get_lane_shape(lane_id)
        if shape:
            pts.extend(shape)
    if not pts:
        return None
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))

def initialize_tls_neighbors():
    global tls_neighbors, tls_controlled_lanes_map
    tls_ids = list(traci.trafficlight.getIDList())
    positions = {}
    tls_controlled_lanes_map = {}

    for tls_id in tls_ids:
        positions[tls_id] = get_tls_centroid(tls_id)
        tls_controlled_lanes_map[tls_id] = set(safe(lambda: traci.trafficlight.getControlledLanes(tls_id), []))

    tls_neighbors = {tls_id: set() for tls_id in tls_ids}
    for i, tls_a in enumerate(tls_ids):
        for tls_b in tls_ids[i+1:]:
            if euclidean_distance(positions.get(tls_a), positions.get(tls_b)) <= SPLIT_TLS_NEIGHBOR_DISTANCE:
                tls_neighbors[tls_a].add(tls_b)
                tls_neighbors[tls_b].add(tls_a)

def lane_is_controlled_by_neighbor(lane_id: str, tls_id: str) -> bool:
    if not lane_id:
        return False
    for nb in tls_neighbors.get(tls_id, set()):
        if lane_id in tls_controlled_lanes_map.get(nb, set()):
            return True
    return False

def apply_network_box_guard(step: int):
    pass

# =========================================================
# GESTIÓN GLOBAL DE TRÁFICO
# =========================================================
def manage_global_traffic(step: int):
    global global_traffic_state, congested_edges_memory

    if not _ENABLE_REROUTE:
        return
    if not ENABLE_GLOBAL_REROUTE_BEFORE_ACCIDENT:
        if not accidente_generado:
            return
        if accident_time_real is None or step < accident_time_real + GLOBAL_REROUTE_START_DELAY_AFTER_ACCIDENT:
            return
    if step % TRAFFIC_MONITOR_INTERVAL != 0:
        return

    current_congested = set()
    for edge_id in traci.edge.getIDList():
        if not is_valid_normal_edge(edge_id):
            continue
        if edge_id == edge_accidente:
            continue
        if is_edge_in_protected_corridor(edge_id):
            continue
        halting = safe_get_edge_halting(edge_id)
        if halting >= CONGESTION_HALTING_THRESHOLD:
            current_congested.add(edge_id)

    if global_traffic_state == TrafficState.NORMAL and current_congested:
        global_traffic_state = TrafficState.CONGESTION_DETECTED

    elif global_traffic_state == TrafficState.CONGESTION_DETECTED:
        global_traffic_state = TrafficState.REROUTING

    elif global_traffic_state == TrafficState.REROUTING and not current_congested:
        global_traffic_state = TrafficState.NORMAL

    congested_edges_memory = current_congested

    if global_traffic_state == TrafficState.REROUTING:
        apply_dynamic_rerouting(step)

def apply_dynamic_rerouting(step: int):
    if not congested_edges_memory:
        return

    protected_corridor = get_ambulance_protected_corridor()
    accident_exclusion = get_accident_exclusion_edges()
    forbidden_edges = protected_corridor.union(accident_exclusion)

    rerouted_count = 0

    for veh_id in traci.vehicle.getIDList():
        if rerouted_count >= MAX_REROUTES_PER_CYCLE:
            break
        if veh_id in (AMBULANCE_ID, vehiculo_accidentado):
            continue
        if not is_vehicle_safe_to_reroute(veh_id):
            continue

        road_id = safe_get_vehicle_road(veh_id)
        if road_id in forbidden_edges:
            continue

        route = safe_get_vehicle_route(veh_id)
        idx = safe_get_vehicle_route_index(veh_id)
        if not route or idx < 0:
            continue

        future_route = list(route[idx:])

        if set(future_route).intersection(forbidden_edges):
            continue
        if not set(future_route).intersection(congested_edges_memory):
            continue

        if random.random() < REROUTE_PROBABILITY:
            try:
                traci.vehicle.rerouteTraveltime(veh_id)
                rerouted_count += 1
            except traci.TraCIException:
                pass

# =========================================================
# ACCIDENTE
# =========================================================
def is_passenger_candidate_multilane(veh_id: str) -> bool:
    if safe_get_vehicle_class(veh_id) != "passenger":
        return False

    road_id = safe_get_vehicle_road(veh_id)
    if not is_valid_normal_edge(road_id) or safe_get_edge_lane_number(road_id) < 2:
        return False

    lane_id = safe_get_vehicle_lane_id(veh_id)
    pos = safe_get_vehicle_lane_pos(veh_id)
    lane_length = safe_get_lane_length(lane_id) if lane_id else None
    if pos is None or lane_length is None:
        return False

    if not (ACCIDENT_MIN_POS_FROM_START <= pos <= lane_length - ACCIDENT_MIN_POS_FROM_END):
        return False

    return True

def is_good_accident_location_from_base(veh_id: str) -> bool:
    road_id = safe_get_vehicle_road(veh_id)
    if not is_valid_normal_edge(road_id):
        return False

    base_edge = get_primary_base_edge()
    if base_edge is None:
        return True

    base_centroid = get_edge_centroid(base_edge)
    accident_centroid = get_edge_centroid(road_id)
    if euclidean_distance(base_centroid, accident_centroid) < ACCIDENT_MIN_DISTANCE_FROM_BASE:
        return False

    try:
        route = traci.simulation.findRoute(base_edge, road_id, vType=AMBULANCE_TYPE_ID)
        if not route or len(route.edges) < MIN_ACC_ROUTE_LENGTH_FROM_BASE:
            return False
    except traci.TraCIException:
        return False

    return True

def choose_accident_vehicle():
    vehs = traci.vehicle.getIDList()
    candidates = [
        v for v in vehs
        if safe_get_vehicle_speed(v) > ACCIDENT_MIN_SPEED
        and is_passenger_candidate_multilane(v)
        and is_good_accident_location_from_base(v)
    ]
    return random.choice(candidates) if candidates else None

def evaluate_accident_tls_context():
    global accident_near_tls, accident_tls_ids

    if edge_accidente is None or lane_accidente is None:
        accident_near_tls, accident_tls_ids = False, []
        return

    lane_id = lane_id_from_edge_and_index(edge_accidente, lane_accidente)
    tls_ids = [
        t for t in traci.trafficlight.getIDList()
        if lane_id in safe(lambda: traci.trafficlight.getControlledLanes(t), [])
    ]
    lane_len = safe_get_lane_length(lane_id)
    dist_to_end = lane_len - pos_accidente if lane_len is not None and pos_accidente is not None else None

    if tls_ids and dist_to_end is not None and dist_to_end <= ACCIDENT_TLS_NEAR_DISTANCE:
        accident_near_tls, accident_tls_ids = True, tls_ids
    else:
        accident_near_tls, accident_tls_ids = False, []

def generate_random_accident(step: int):
    global accidente_generado, vehiculo_accidentado, edge_accidente
    global lane_accidente, pos_accidente, accident_time_real, accident_route_from_base, ambulance_target_pos

    veh_id = choose_accident_vehicle()
    if veh_id is None:
        return False

    road_id = safe_get_vehicle_road(veh_id)
    lane_index = safe_get_vehicle_lane_index(veh_id)
    lane_id = safe_get_vehicle_lane_id(veh_id)
    pos = safe_get_vehicle_lane_pos(veh_id)
    lane_length = safe_get_lane_length(lane_id)

    if None in (road_id, lane_index, lane_id, pos, lane_length):
        return False

    speed = safe_get_vehicle_speed(veh_id)
    decel = safe(lambda: traci.vehicle.getDecel(veh_id), 4.5)
    braking_distance = (speed ** 2) / max(2 * decel, 0.1) + 3.0
    pos_safe = pos + braking_distance

    if pos_safe > lane_length - 8.0:
        return False

    try:
        traci.vehicle.setStop(
            vehID=veh_id,
            edgeID=road_id,
            pos=pos_safe,
            laneIndex=lane_index,
            duration=float(ACCIDENT_DURATION)
        )
        traci.vehicle.setColor(veh_id, (255, 0, 0, 255))

        accidente_generado = True
        vehiculo_accidentado = veh_id
        edge_accidente = road_id
        lane_accidente = lane_index
        pos_accidente = pos_safe
        ambulance_target_pos = max(8.0, pos_accidente - AMBULANCE_TARGET_OFFSET)
        accident_time_real = step

        base_edge = get_primary_base_edge()
        if base_edge is not None:
            route = safe(lambda: traci.simulation.findRoute(base_edge, road_id, vType=AMBULANCE_TYPE_ID))
            accident_route_from_base = list(route.edges) if route else []
        else:
            accident_route_from_base = []

        evaluate_accident_tls_context()
        log_event(
            f"\n[{step}s] ACCIDENTE GENERADO | veh={veh_id} | edge={road_id} | "
            f"TLS={accident_near_tls} | target_amb={ambulance_target_pos:.2f}"
        )
        return True

    except traci.TraCIException as e:
        return False

# =========================================================
# DESPACHO DE AMBULANCIA
# =========================================================
def set_ambulance_state(new_state: str, step: int, reason: str = ""):
    global ambulance_state
    if ambulance_state != new_state:
        log_event(f"[{step}s] [AMB-FSM] {ambulance_state} -> {new_state}. {reason}".strip())
        ambulance_state = new_state

def dispatch_ambulance(step: int):
    global ambulancia_creada, ambulancia_despachada, ambulancia_edge_origen
    global last_route_id, ambulance_dispatch_failed

    if edge_accidente is None:
        return False

    if ambulance_dispatch_failed:
        return False

    if AMBULANCE_ID in traci.vehicle.getIDList():
        ambulancia_creada = True
        ambulancia_despachada = True
        return True

    origin, route_edges = choose_fixed_ambulance_origin(edge_accidente)
    if origin is None or not route_edges:
        log_event(f"[{step}s] No se encontró ruta válida para ambulancia hacia {edge_accidente}.")
        ambulance_dispatch_failed = True
        return False

    ambulancia_edge_origen = origin
    unique_route_id = f"ruta_amb_{step}"
    last_route_id = unique_route_id

    try:
        if unique_route_id not in traci.route.getIDList():
            traci.route.add(unique_route_id, route_edges)

        traci.vehicle.add(
            vehID=AMBULANCE_ID,
            routeID=unique_route_id,
            typeID=AMBULANCE_TYPE_ID,
            depart="now",
            departLane="free",
            departPos="free"
        )

        traci.vehicle.setColor(AMBULANCE_ID, (255, 255, 255, 255))
        traci.vehicle.setVehicleClass(AMBULANCE_ID, "emergency")
        traci.vehicle.setShapeClass(AMBULANCE_ID, "emergency")
        
        # LÓGICA CORREGIDA DE CONDUCCIÓN
        if _ENABLE_TLS:
            # Modo controlado: Ignora semáforos y ceda el paso (total nosotros le ponemos el verde)
            traci.vehicle.setSpeedMode(AMBULANCE_ID, 7)
        else:
            # Modo normal: 31 es el valor por defecto en SUMO (respeta semáforos y señales)
            traci.vehicle.setSpeedMode(AMBULANCE_ID, 31)
            
        traci.vehicle.setLaneChangeMode(AMBULANCE_ID, 597)
        traci.vehicle.setParameter(AMBULANCE_ID, "laneChangeModel", "SL2015")
        traci.vehicle.setParameter(AMBULANCE_ID, "laneChangeModel.lcStrategic", "50")
        traci.vehicle.setParameter(AMBULANCE_ID, "laneChangeModel.lcCooperative", "1")
        traci.vehicle.setParameter(AMBULANCE_ID, "laneChangeModel.lcSpeedGain", "40")
        traci.vehicle.setParameter(AMBULANCE_ID, "laneChangeModel.lcKeepRight", "0")

        ambulancia_creada = True
        ambulancia_despachada = True
        set_ambulance_state(AmbulanceState.DISPATCHED, step, f"origen={origin}")
        log_event(f"[{step}s] AMBULANCIA DESPACHADA | origen={origin} | ruta={route_edges}")
        return True

    except traci.TraCIException as e:
        if AMBULANCE_ID in traci.vehicle.getIDList():
            ambulancia_creada = True
            ambulancia_despachada = True
            return True

        ambulance_dispatch_failed = True
        return False

# =========================================================
# PRIORIDAD SEMAFÓRICA Y DRENAJE
# =========================================================
def remember_tls_original_state(tls_id: str):
    if tls_id not in tls_original_program:
        prog = safe(lambda: traci.trafficlight.getProgram(tls_id))
        phase = safe(lambda: traci.trafficlight.getPhase(tls_id))
        if prog is not None and phase is not None:
            tls_original_program[tls_id] = (prog, phase)

def restore_tls_program(tls_id: str):
    if tls_id in tls_original_program:
        try:
            traci.trafficlight.setProgram(tls_id, tls_original_program[tls_id][0])
            traci.trafficlight.setPhase(tls_id, tls_original_program[tls_id][1])
        except traci.TraCIException:
            pass

def build_green_state_for_incoming_lane(tls_id: str, incoming_lane: str):
    state = safe(lambda: traci.trafficlight.getRedYellowGreenState(tls_id))
    links = safe(lambda: traci.trafficlight.getControlledLinks(tls_id), [])
    if not state or not links:
        return None

    new_state = ["r"] * len(state)
    for i, link_group in enumerate(links):
        for link in link_group:
            if link[0] == incoming_lane:
                new_state[i] = "G"
    if "G" not in new_state:
        return None
    return "".join(new_state)

def clear_intersection_box(tls_id: str, incoming_lane: str, ambulance_id: str, strong: bool = False):
    controlled_lanes = set(safe(lambda: traci.trafficlight.getControlledLanes(tls_id), []))
    stop_dist = TLS_STOP_DIST_STRONG if strong else TLS_STOP_DIST

    for veh_id in traci.vehicle.getIDList():
        if veh_id in (ambulance_id, vehiculo_accidentado):
            continue

        lane_id = safe_get_vehicle_lane_id(veh_id)
        if lane_id is None or lane_id not in controlled_lanes:
            continue
        if is_internal_lane(lane_id):
            continue

        lane_pos = safe_get_vehicle_lane_pos(veh_id)
        lane_len = safe_get_lane_length(lane_id)
        current_speed = safe_get_vehicle_speed(veh_id)

        if lane_pos is None or lane_len is None:
            continue

        dist_to_stop = lane_len - lane_pos

        try:
            if lane_id == incoming_lane:
                traci.vehicle.slowDown(
                    veh_id,
                    max(current_speed + 2.0, 7.0 if not strong else 9.0),
                    2
                )
            elif dist_to_stop <= stop_dist:
                traci.vehicle.slowDown(veh_id, 0.1, 2)
        except traci.TraCIException:
            pass

def preclear_destination_intersection(step: int):
    global total_tls_interventions
    if not accident_near_tls or not accident_tls_ids:
        return
    if not ambulancia_creada or AMBULANCE_ID not in traci.vehicle.getIDList():
        return
    if ambulancia_llego:
        return

    amb_edge = safe_get_vehicle_road(AMBULANCE_ID)
    amb_lane = safe_get_vehicle_lane_id(AMBULANCE_ID)
    if amb_edge is None or amb_lane is None:
        return
    if is_internal_edge(amb_edge) or is_internal_lane(amb_lane):
        return

    next_tls = safe(lambda: traci.vehicle.getNextTLS(AMBULANCE_ID), [])
    if not next_tls:
        return

    for item in next_tls[:TLS_SCAN_AHEAD]:
        try:
            tls_id, tls_index, dist = item[0], int(item[1]), float(item[2])
        except Exception:
            continue

        if tls_id not in accident_tls_ids:
            continue
        if dist > DESTINATION_TLS_PREEMPTION_DISTANCE:
            continue

        links = safe(lambda: traci.trafficlight.getControlledLinks(tls_id), [])
        if not links or tls_index < 0 or tls_index >= len(links):
            continue

        incoming_lane = next((link[0] for link in links[tls_index]), None)
        if incoming_lane is None:
            continue

        last = tls_cooldown.get(tls_id, -999999)
        if step - last < TLS_MIN_REAPPLY_GAP:
            continue

        desired_state = build_green_state_for_incoming_lane(tls_id, incoming_lane)
        if desired_state is None:
            continue

        try:
            remember_tls_original_state(tls_id)
            traci.trafficlight.setRedYellowGreenState(tls_id, desired_state)
            tls_priority_memory[tls_id] = TLS_HOLD_STEPS + 8
            tls_cooldown[tls_id] = step
            total_tls_interventions += 1
            clear_intersection_box(tls_id, incoming_lane, AMBULANCE_ID, strong=True)
            boost_ambulance_speed(AMBULANCE_ID, strong=True)
            set_ambulance_state(AmbulanceState.PRECLEARING_TLS, step, f"tls={tls_id}")
        except traci.TraCIException:
            pass

def apply_tls_preemption_for_ambulance(step: int, ambulance_id: str):
    global total_tls_interventions
    if not _ENABLE_TLS:
        return
    
    if ambulance_id not in traci.vehicle.getIDList():
        return

    amb_edge = safe_get_vehicle_road(ambulance_id)
    amb_lane = safe_get_vehicle_lane_id(ambulance_id)

    if amb_edge is None or amb_lane is None:
        return
    if is_internal_edge(amb_edge) or is_internal_lane(amb_lane):
        return

    next_tls = safe(lambda: traci.vehicle.getNextTLS(ambulance_id), [])
    if not next_tls:
        return

    for item in next_tls[:TLS_SCAN_AHEAD]:
        try:
            tls_id, tls_index, dist = item[0], int(item[1]), float(item[2])
        except Exception:
            continue

        links = safe(lambda: traci.trafficlight.getControlledLinks(tls_id), [])
        state = safe(lambda: traci.trafficlight.getRedYellowGreenState(tls_id))
        if not links or state is None or tls_index < 0 or tls_index >= len(links):
            continue

        incoming_lane = next((link[0] for link in links[tls_index]), None)
        if incoming_lane is None:
            continue

        last = tls_cooldown.get(tls_id, -999999)
        if step - last < TLS_MIN_REAPPLY_GAP:
            continue

        strong = accident_near_tls and (tls_id in accident_tls_ids)
        trigger_distance = DESTINATION_TLS_PREEMPTION_DISTANCE if strong else TLS_PREEMPTION_DISTANCE
        if dist > trigger_distance:
            continue

        desired_state = build_green_state_for_incoming_lane(tls_id, incoming_lane)
        if desired_state is None or state == desired_state:
            continue

        try:
            remember_tls_original_state(tls_id)
            traci.trafficlight.setRedYellowGreenState(tls_id, desired_state)
            tls_priority_memory[tls_id] = TLS_HOLD_STEPS + (10 if strong else 0)
            tls_cooldown[tls_id] = step
            total_tls_interventions += 1
            clear_intersection_box(tls_id, incoming_lane, ambulance_id, strong=strong)
            boost_ambulance_speed(ambulance_id, strong=strong)
        except traci.TraCIException:
            pass

def maintain_or_release_tls_priority():
    for tls_id in list(tls_priority_memory.keys()):
        if tls_priority_memory[tls_id] <= 0:
            restore_tls_program(tls_id)
            del tls_priority_memory[tls_id]
        else:
            tls_priority_memory[tls_id] -= 1

# =========================================================
# COOPERACIÓN VEHICULAR (LIMPIA Y SEGURA)
# =========================================================
def cooperative_clear_path_for_ambulance(ambulance_id: str):
    if not _ENABLE_COOP:
        return
    
    amb_edge = safe_get_vehicle_road(ambulance_id)
    amb_lane_id = safe_get_vehicle_lane_id(ambulance_id)
    amb_lane_idx = safe_get_vehicle_lane_index(ambulance_id)
    amb_pos = safe_get_vehicle_lane_pos(ambulance_id)
    amb_speed = safe_get_vehicle_speed(ambulance_id)

    if None in (amb_edge, amb_lane_id, amb_lane_idx, amb_pos, amb_speed):
        return
    if is_internal_edge(amb_edge) or is_internal_lane(amb_lane_id):
        return

    for veh_id in traci.vehicle.getIDList():
        if veh_id == ambulance_id:
            continue

        road_id = safe_get_vehicle_road(veh_id)
        lane_id = safe_get_vehicle_lane_id(veh_id)
        lane_pos = safe_get_vehicle_lane_pos(veh_id)
        current_lane_idx = safe_get_vehicle_lane_index(veh_id)

        if road_id != amb_edge or None in (lane_id, lane_pos, current_lane_idx):
            continue
        if is_internal_edge(road_id) or is_internal_lane(lane_id):
            continue

        delta = lane_pos - amb_pos

        try:
            if 0 < delta <= CLEARING_DISTANCE_AHEAD:
                if current_lane_idx == amb_lane_idx:
                    traci.vehicle.slowDown(veh_id, max(amb_speed + 1.5, 4.0), 2)
                else:
                    traci.vehicle.slowDown(veh_id, SLOWDOWN_SPEED, 2)
            elif -CLEARING_DISTANCE_BEHIND <= delta < 0:
                traci.vehicle.slowDown(veh_id, SLOWDOWN_SPEED, 2)
        except traci.TraCIException:
            pass

# =========================================================
# DETECTOR DE ATASCO Y RECUPERACIÓN GRADUAL
# =========================================================
def update_ambulance_history(step: int):
    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return
    ambulance_history.append({
        "step": step,
        "edge": safe_get_vehicle_road(AMBULANCE_ID),
        "lane": safe_get_vehicle_lane_id(AMBULANCE_ID),
        "pos": safe_get_vehicle_lane_pos(AMBULANCE_ID),
        "speed": safe_get_vehicle_speed(AMBULANCE_ID),
        "wait": safe_get_vehicle_waiting_time(AMBULANCE_ID),
    })

def is_ambulance_really_stuck():
    if len(ambulance_history) < AMB_HISTORY_LEN:
        return False

    first = ambulance_history[0]
    last = ambulance_history[-1]

    if first["edge"] != last["edge"] or first["lane"] != last["lane"]:
        return False
    if first["pos"] is None or last["pos"] is None:
        return False

    delta_pos = abs(last["pos"] - first["pos"])
    avg_speed = sum(x["speed"] or 0 for x in ambulance_history) / len(ambulance_history)
    return delta_pos < STUCK_MIN_PROGRESS and avg_speed < STUCK_MAX_AVG_SPEED

def remove_primary_blocker_ahead():
    # SI EL TELETRANSPORTE ESTÁ DESACTIVADO, NO HACEMOS MAGIA
    if not _ENABLE_TELEPORT:
        return False

    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return False

    amb_edge = safe_get_vehicle_road(AMBULANCE_ID)
    amb_lane = safe_get_vehicle_lane_index(AMBULANCE_ID)
    amb_lane_id = safe_get_vehicle_lane_id(AMBULANCE_ID)
    amb_pos = safe_get_vehicle_lane_pos(AMBULANCE_ID)
    if None in (amb_edge, amb_lane, amb_lane_id, amb_pos):
        return False
    if is_internal_edge(amb_edge) or is_internal_lane(amb_lane_id):
        return False

    best_blocker = None
    best_delta = float("inf")

    for veh_id in traci.vehicle.getIDList():
        if veh_id in (AMBULANCE_ID, vehiculo_accidentado):
            continue

        veh_edge = safe_get_vehicle_road(veh_id)
        veh_lane_id = safe_get_vehicle_lane_id(veh_id)
        if veh_edge != amb_edge:
            continue
        if is_internal_edge(veh_edge) or is_internal_lane(veh_lane_id):
            continue
        if safe_get_vehicle_lane_index(veh_id) != amb_lane:
            continue

        v_pos = safe_get_vehicle_lane_pos(veh_id)
        if v_pos is None:
            continue
        delta = v_pos - amb_pos
        if 5.0 < delta <= BLOCKER_REMOVE_DISTANCE and delta < best_delta:
            best_delta = delta
            best_blocker = veh_id

    if best_blocker is not None:
        safe_remove_vehicle(best_blocker)
        log_event(f"[ANTI-DEADLOCK] Bloqueador retirado: {best_blocker}")
        return True
    return False

def handle_ambulance_recovery(step: int):
    global last_stuck_resolution_step, ambulance_recovery_level

    if not ambulancia_creada or ambulancia_llego:
        return
    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return
    if not is_ambulance_really_stuck():
        if ambulance_recovery_level != 0:
            set_ambulance_state(AmbulanceState.EN_ROUTE, step, "Recuperada del atasco")
        ambulance_recovery_level = 0
        return
    if step - last_stuck_resolution_step < STUCK_RESOLVE_COOLDOWN:
        return

    last_stuck_resolution_step = step
    ambulance_recovery_level = min(3, ambulance_recovery_level + 1)

    if ambulance_recovery_level == 1:
        set_ambulance_state(AmbulanceState.STUCK_L1, step, "Ambulancia atascada, intentando acelerar...")
        reroute_ambulance_if_needed()
        boost_ambulance_speed(AMBULANCE_ID, strong=True)
        cooperative_clear_path_for_ambulance(AMBULANCE_ID)

    elif ambulance_recovery_level == 2:
        set_ambulance_state(AmbulanceState.STUCK_L2, step, "Atasco severo...")
        removed = False
        if _ENABLE_TELEPORT:
            removed = remove_primary_blocker_ahead()
        if not removed:
            reroute_ambulance_if_needed()

    elif ambulance_recovery_level >= 3:
        if ambulance_is_too_close_to_junction():
            set_ambulance_state(AmbulanceState.STUCK_L2, step, "Cerca de nodo/TLS, esperando.")
            if _ENABLE_TELEPORT:
                remove_primary_blocker_ahead()
            reroute_ambulance_if_needed()
            return

        set_ambulance_state(AmbulanceState.STUCK_L3, step, "Atasco crítico...")
        if _ENABLE_TELEPORT:
            remove_primary_blocker_ahead()

# =========================================================
# ZONA DEL ACCIDENTE
# =========================================================
def mitigate_accident_zone():
    if not accidente_generado:
        return

    for veh_id in traci.vehicle.getIDList():
        if veh_id in (vehiculo_accidentado, AMBULANCE_ID):
            continue
        if safe_get_vehicle_road(veh_id) != edge_accidente:
            continue

        lane_idx = safe_get_vehicle_lane_index(veh_id)
        pos = safe_get_vehicle_lane_pos(veh_id)
        speed = safe_get_vehicle_speed(veh_id)
        if lane_idx is None or pos is None:
            continue

        if not (0 < pos_accidente - pos <= ACCIDENT_ZONE_DISTANCE):
            continue

        try:
            if lane_idx == lane_accidente:
                traci.vehicle.slowDown(veh_id, max(speed + 1.0, 4.0), 2)
            else:
                traci.vehicle.slowDown(veh_id, max(speed + 1.5, 5.0), 2)
        except traci.TraCIException:
            pass

def teleport_blockers_single_lane_for_ambulance():
    # SI EL TELETRANSPORTE ESTÁ DESACTIVADO, SALIMOS INMEDIATAMENTE
    if not _ENABLE_TELEPORT:
        return

    if not ambulancia_creada or ambulancia_llego:
        return
    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return
    if ambulance_is_in_internal():
        return

    amb_wait = safe_get_vehicle_waiting_time(AMBULANCE_ID)
    if amb_wait < AMBULANCE_WAIT_BEFORE_BLOCKER_REMOVE:
        return

    remaining_route = list(get_remaining_route_edges(AMBULANCE_ID))
    if not remaining_route:
        return

    candidate_edges = set(list(remaining_route[:2]) + ([edge_accidente] if edge_accidente else []))
    amb_edge = safe_get_vehicle_road(AMBULANCE_ID)
    amb_lane_id = safe_get_vehicle_lane_id(AMBULANCE_ID)
    amb_lane_pos = safe_get_vehicle_lane_pos(AMBULANCE_ID)

    removed = 0
    for veh_id in list(traci.vehicle.getIDList()):
        if removed >= MAX_TELEPORTS_PER_CALL:
            break
        if veh_id in (vehiculo_accidentado, AMBULANCE_ID):
            continue

        road_id = safe_get_vehicle_road(veh_id)
        lane_id = safe_get_vehicle_lane_id(veh_id)
        lane_pos = safe_get_vehicle_lane_pos(veh_id)
        waiting = safe_get_vehicle_waiting_time(veh_id)
        speed = safe_get_vehicle_speed(veh_id)

        if road_id not in candidate_edges:
            continue
        if is_internal_edge(road_id) or is_internal_lane(lane_id):
            continue
        if road_id == amb_edge and lane_id != amb_lane_id:
            continue
        if road_id == amb_edge and amb_lane_pos is not None and lane_pos is not None:
            delta = lane_pos - amb_lane_pos
            if delta <= 5.0 or delta > SINGLE_LANE_RELIEF_DISTANCE:
                continue

        if waiting >= TELEPORT_WAIT_THRESHOLD and speed <= 0.2:
            safe_remove_vehicle(veh_id)
            removed += 1
            log_event(f"[TELEPORT] Bloqueador retirado | veh={veh_id} | edge={road_id}")

# =========================================================
# LLEGADA Y LIBERACIÓN
# =========================================================
def check_ambulance_arrival(step: int):
    global ambulancia_llego, ambulancia_arrival_time

    if not ambulancia_creada or ambulancia_llego or AMBULANCE_ID not in traci.vehicle.getIDList():
        return

    amb_edge = safe_get_vehicle_road(AMBULANCE_ID)
    amb_lane = safe_get_vehicle_lane_index(AMBULANCE_ID)
    amb_lane_id = safe_get_vehicle_lane_id(AMBULANCE_ID)
    amb_pos = safe_get_vehicle_lane_pos(AMBULANCE_ID)
    if None in (amb_edge, amb_lane, amb_lane_id, amb_pos, edge_accidente, ambulance_target_pos):
        return
    if is_internal_edge(amb_edge) or is_internal_lane(amb_lane_id):
        return

    lane_ok = (amb_lane == lane_accidente) or (safe_get_edge_lane_number(edge_accidente) > 1)
    if amb_edge == edge_accidente and lane_ok and abs(amb_pos - ambulance_target_pos) <= ARRIVAL_TOLERANCE:
        ambulancia_llego = True
        ambulancia_arrival_time = step
        try:
            traci.vehicle.setColor(AMBULANCE_ID, (0, 255, 0, 255))
        except traci.TraCIException:
            pass
        set_ambulance_state(AmbulanceState.ON_SCENE, step, "Ambulancia en escena")
        log_event(f"\n[ÉXITO] AMBULANCIA LLEGÓ | t={step}s | amb_pos={amb_pos:.2f} | target={ambulance_target_pos:.2f}")

def clear_accident_after_ambulance(step: int):
    global accidente_liberado
    if not ambulancia_llego or accidente_liberado or ambulancia_arrival_time is None:
        return
    if step < ambulancia_arrival_time + ACCIDENT_CLEAR_DELAY:
        return

    for v_id in [vehiculo_accidentado, AMBULANCE_ID]:
        if v_id in traci.vehicle.getIDList():
            safe_remove_vehicle(v_id)
    
    accidente_liberado = True
    set_ambulance_state(AmbulanceState.CLEARED, step, "Incidente liberado")

# =========================================================
# DEBUG DE AMBULANCIA
# =========================================================
def debug_ambulance_state(step: int):
    if not DEBUG or AMBULANCE_ID not in traci.vehicle.getIDList():
        return

    road_id = safe_get_vehicle_road(AMBULANCE_ID)
    lane_id = safe_get_vehicle_lane_id(AMBULANCE_ID)
    lane_idx = safe_get_vehicle_lane_index(AMBULANCE_ID)
    pos = safe_get_vehicle_lane_pos(AMBULANCE_ID)
    speed = safe_get_vehicle_speed(AMBULANCE_ID)
    wait = safe_get_vehicle_waiting_time(AMBULANCE_ID)
    route_idx = safe_get_vehicle_route_index(AMBULANCE_ID)
    next_tls = safe(lambda: traci.vehicle.getNextTLS(AMBULANCE_ID), [])

    log_event(
        f"[AMB-DEBUG] t={step} | state={ambulance_state} | edge={road_id} | lane={lane_id} | "
        f"lane_idx={lane_idx} | pos={pos} | speed={speed:.2f} | wait={wait:.2f} | "
        f"route_idx={route_idx} | next_tls={next_tls[:1]}"
    )

# =========================================================
# MAIN
# =========================================================
def main():
    global ambulance_base_candidates

    traci.start([
        SUMO_BINARY, 
        "-c", SUMO_CFG,
        "--ignore-junction-blocker", "10", 
        "--time-to-teleport", "45"         
    ])

    ambulance_base_candidates = compute_ambulance_base_candidates()
    initialize_tls_neighbors()
    log_event(f"Semilla usada: {RANDOM_SEED}")
    log_event(f"ACCIDENT_TIME: {ACCIDENT_TIME}")
    log_event(f"[BASE] Candidatos de ambulancia: {ambulance_base_candidates}")

    step = 0
    try:
        while step <= SIM_DURATION:
            traci.simulationStep()

            manage_global_traffic(step)
            apply_network_box_guard(step)

            if (step >= ACCIDENT_TIME) and (not accidente_generado):
                generate_random_accident(step)

            if (
                accidente_generado and not ambulancia_despachada and accident_time_real is not None
                and step >= accident_time_real + AMBULANCE_DELAY
            ):
                dispatch_ambulance(step)

            mitigate_accident_zone()

            if ambulancia_creada and AMBULANCE_ID in traci.vehicle.getIDList() and not ambulancia_llego:
                if ambulance_state == AmbulanceState.DISPATCHED:
                    set_ambulance_state(AmbulanceState.EN_ROUTE, step, "Ambulancia en camino")

                update_ambulance_history(step)
                preclear_destination_intersection(step)
                apply_tls_preemption_for_ambulance(step, AMBULANCE_ID)
                cooperative_clear_path_for_ambulance(AMBULANCE_ID)
                handle_ambulance_recovery(step)

                # Eliminado el bypass de teletransporte. Ahora handle_ambulance_recovery llama a lo seguro.
                if ambulance_recovery_level >= 1 and is_ambulance_really_stuck():
                    teleport_blockers_single_lane_for_ambulance()

                if DEBUG and step % 20 == 0:
                    debug_ambulance_state(step)

            maintain_or_release_tls_priority()

            if ambulancia_creada and not ambulancia_llego:
                check_ambulance_arrival(step)
            clear_accident_after_ambulance(step)

            step += 1

    except traci.exceptions.FatalTraCIError as e:
        log_event(f"\n[ERROR FATAL C++] SUMO se cerró inesperadamente en el paso {step}.")
    finally:
        try:
            traci.close()
        except Exception:
            pass
        
        print("\n" + "#" * 78)
        print("FIN DE SIMULACIÓN")
        print(f"Ambulancia llegó          : {ambulancia_llego}")
        print(f"Incidente liberado        : {accidente_liberado}")
        print("#" * 78)
        
        save_simulation_metrics()

def save_simulation_metrics():
    """Guarda métricas de la simulación en JSON"""
    if not _OUTPUT_METRICS:
        return
    
    metrics = {
        "mode": _CONFIG_MODE,
        "simulation_duration": SIM_DURATION,
        "random_seed": RANDOM_SEED,
        "enable_tls_control": _ENABLE_TLS,
        "enable_vehicle_cooperation": _ENABLE_COOP,
        "enable_teleports": _ENABLE_TELEPORT, # Añadido para tener registro
        "ambulance_arrived": ambulancia_llego,
        "ambulance_arrival_time": ambulancia_arrival_time,
        "accident_generated": accidente_generado,
        "accident_time": accident_time_real,
        "incident_cleared": accidente_liberado,
        "ambulance_dispatch_failed": ambulance_dispatch_failed,
        "recovery_level": ambulance_recovery_level,
        "total_tls_preemptions": total_tls_interventions, # Usando la variable corregida
        "total_events_logged": len(event_log),
    }
    
    if accidente_generado and ambulancia_llego and ambulancia_arrival_time and accident_time_real:
        metrics["response_time"] = ambulancia_arrival_time - accident_time_real
    
    metrics_file = Path(__file__).parent / _METRICS_FILE
    
    try:
        with open(metrics_file, 'w') as f:
            json.dump(metrics, f, indent=2)
        log_event(f"[METRICS] Guardadas en {metrics_file}")
    except Exception as e:
        log_event(f"[ERROR] Guardando métricas: {e}")

if __name__ == "__main__":
    main()