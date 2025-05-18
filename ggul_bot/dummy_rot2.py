import time
import odrive
from odrive.enums import *
import math
import can
import struct
from enum import IntEnum
import json

# CAN ID 생성 함수
def generate_can_id(node_id: int, command_id: int) -> int:
    
    return (node_id << 5) + command_id

# 4바이트 16진수의 CAN Message 생성 후 최종 8바이트로 합성 
def generate_can_message(*values: float) -> bytes:

    float_list = list(values)

    # 0으로 자동 패딩(8바이트)
    while len(float_list) * 4 < 8:
        float_list.append(0.0)

    payload = b''.join(struct.pack('<f', v) for v in float_list)

    if len(payload) > 8:
        raise ValueError("CAN 데이터는 최대 8바이트만 허용됩니다.")

    return payload

# Node ID 목록
class NodeID(IntEnum):
    NODE_0 = 0 #odrv0.axis0
    NODE_1 = 1 #odrv0.axis1
    NODE_2 = 2 #odrv1.axis0
    NODE_3 = 3 #odrv1.axis1
    NODE_4 = 4 #odrv2.axis0
    NODE_5 = 5 #odrv2.axis1

# CAN 명령어 목록     
class CommandID(IntEnum):
    Set_Axis_State = 0x07 #(0byte - Axis_Requested_State)
    Get_Encoder_Estimates = 0x09 #(0byte - Pos_Estimate / 4byte - Vel_Estimate)
    Set_Controller_Mode = 0x0b #(0byte - Control_Mode / 4byte - Input_Mode)
    Set_Input_Pos = 0x0c #(0byte - Input_pos / 4byte - Vel FF / 6byte - Torque_FF)
    Set_Traj_Vel_Limit = 0x11 #(0byte - Traj_Vel_Limit)
    Set_Traj_Accel_Limits = 0x12 #(0byte - Traj_Accel_Limit / 4byte - Traj_Decel_Limit)

# odrv0.axis0.encoder.pos_estimate 함수(CAN.ver)
def get_pos_estimate(bus: can.Bus, node_id: int) -> float:
    can_id = generate_can_id(node_id, CommandID.Get_Encoder_Estimates)


    # 위치 요청 전송
    msg = can.Message(arbitration_id=can_id, data=[], is_extended_id=False)
    try:
        bus.send(msg)
        print(f"[전송] Node {node_id} → 위치 요청 (CAN ID: 0x{can_id:X})")
    except can.CanError as e:
        raise RuntimeError(f"[에러] CAN 메시지 전송 실패 (Node {node_id}): {e}")

    # 응답 수신 (최대 0.01초 동안 대기)
    start_time = time.time()
    while time.time() - start_time < 0.05:
        rx_msg = bus.recv(timeout=0.05)
        if rx_msg is None:
            continue
        if len(rx_msg.data) < 4:
            continue

        # 응답 데이터만 파싱 (ID는 무시)
        pos_estimate = struct.unpack('<f', rx_msg.data[0:4])[0]
        return pos_estimate

    raise TimeoutError(f"[에러] Node {node_id} 응답 없음")


#  odrv0.axis0.controller.input_pos 함수(CAN.ver)
def send_input_pos(bus: can.Bus, node_id: int, input_pos: float, vel_ff: float = 0.0):
    INPUT_POS_CMD = CommandID.Set_Input_Pos
    can_id = generate_can_id(node_id, INPUT_POS_CMD)

    payload = generate_can_message(input_pos, vel_ff)

    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError as e:
        raise RuntimeError(f"CAN 전송 실패 (Node {node_id}): {e}")

# odrv0.axis0.trap_traj.config.vel_limit 함수(CAN.ver)
def send_traj_vel_limit(bus: can.Bus, node_id: int, vel_limit: float):
    can_id = generate_can_id(node_id, CommandID.Set_Traj_Vel_Limit)
    payload = generate_can_message(vel_limit)
    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError as e:
        raise RuntimeError(f"[Node {node_id}] Traj Vel 전송 실패: {e}")

#  odrv0.axis0.trap_traj.config.accel/decel_limit 함수(CAN.ver)
def send_traj_accel_limits(bus: can.Bus, node_id: int, accel: float, decel: float):
    can_id = generate_can_id(node_id, CommandID.Set_Traj_Accel_Limits)
    payload = generate_can_message(accel, decel)
    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError as e:
        raise RuntimeError(f"[Node {node_id}] Traj Accel 전송 실패: {e}")
        
# 목표 위치 도달 확인 함수
def motors_reached_target(bus: can.Bus, positions, tolerance: float) -> bool:
    all_reached = True
    for i, node_id in enumerate(NodeID):
        try:
            pos = get_pos_estimate(bus, node_id)
            diff = abs(pos - positions[i])
            print(f"[비교] Node {node_id} 현재 위치: {pos:.2f}, 목표: {positions[i]:.2f}, 오차: {diff:.2f}")

            if diff >= tolerance:
                all_reached = False
        except Exception as e:
            print(f"[오류] Node {node_id} 위치 확인 실패: {e}")
            all_reached = False
    return all_reached

# 포지션 제어 모드 설정 함수 (control_mode=3, input_mode=16)
def send_position_control_mode(bus: can.Bus, node_id: int):
    can_id = generate_can_id(node_id, CommandID.Set_Controller_Mode)
    payload = b'\x03\x00\x00\x00' + b'\x05\x00\x00\x00'  # 3 = POSITION_CONTROL, 5 = PASSTHROUGH
    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"[OK] Node {node_id} → POSITION_CONTROL 모드 설정 완료")
    except can.CanError as e:
        raise RuntimeError(f"[Node {node_id}] Control Mode 전송 실패: {e}")

# Closed-loop 상태 진입 함수 (state=8)
def send_closed_loop_state(bus: can.Bus, node_id: int):
    can_id = generate_can_id(node_id, CommandID.Set_Axis_State)
    payload = b'\x08\x00\x00\x00' + b'\x00\x00\x00\x00'  # 8 = CLOSED_LOOP_CONTROL
    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"[OK] Node {node_id} → CLOSED_LOOP_CONTROL 상태 진입 완료")
    except can.CanError as e:
        raise RuntimeError(f"[Node {node_id}] Axis State 전송 실패: {e}")
        
# 각도 -> encoder counts 변환 함수
def angle_to_encoder_counts(angle, reduction_ratio):
    mechanical_revs = angle / 360.0
    encoder_counts = mechanical_revs * reduction_ratio * encoder_ppr
    return encoder_counts

# 가감속 속도 및 가속도 계산 함수
def calculate_trap_traj_speeds(current_positions, target_positions_encoder, max_vel_limit, max_accel_limit):
    distances = [abs(target - current) for target, current in zip(target_positions_encoder, current_positions)]
    max_distance = max(distances)

    if max_distance == 0:
        return [0] * 6, [0] * 6

    speed_limits = [(distance / max_distance) * max_vel_limit for distance in distances]
    accel_limits = [(distance / max_distance) * max_accel_limit for distance in distances]

    return speed_limits, accel_limits



# ================================================
# 조인트 세트 처리 함수
# ================================================
def process_joint_set(joint_angles):
    print(f"\nProcessing joint angles: {joint_angles}")

    if len(joint_angles) < 6:
        print('Not enough joint angles received.')
        return
    
    joint_angles = [math.degrees(angle) for angle in joint_angles]
    print(f"Converted joint angles to degrees: {joint_angles}")

    reduction_ratios = [
        reduction_ratio_0,
        reduction_ratio_1,
        reduction_ratio_2,
        reduction_ratio_3,
        reduction_ratio_4,
        reduction_ratio_5,
    ]

    target_positions = [
        (reduction_ratios[i] / 360) * angle for i, angle in enumerate(joint_angles)
    ]

    target_positions_encoder = [
        angle_to_encoder_counts(joint_angles[i], reduction_ratios[i])
        for i in range(6)
    ]

    current_positions = [get_pos_estimate(bus, node) for node in NodeID]

   # 최대 속도/가속 제한 설정
    max_vel_limit = 9
    max_accel_limit = 8.5

    # current_positions, target_positions_encoder 는 기존에 계산됨
    speed_limits, accel_limits = calculate_trap_traj_speeds(
        current_positions, target_positions_encoder, max_vel_limit, max_accel_limit
    )

    # 각 노드에 속도/가속 설정 전송
    for i, node_id in enumerate(NodeID):
        send_traj_vel_limit(bus, node_id, speed_limits[i])
        send_traj_accel_limits(bus, node_id, accel_limits[i], accel_limits[i])  # accel == decel


    # 목표 위치 명령
    for i, node_id in enumerate(NodeID):
        time.sleep(0.01)  
        send_input_pos(bus, node_id, target_positions[i])
        time.sleep(0.01)

    print("Motors are moving...")

    while not motors_reached_target(bus, target_positions, TOLERANCE):
        time.sleep(0.1)

    print("Motors have reached their target positions.")
    time.sleep(3)

# 1. CAN 인터페이스 연결
try:
    bus = can.interface.Bus(interface='slcan', channel='COM3', bitrate = 250000)  # slcan0 인터페이스 사용
    print("[INFO] CAN 버스 연결 성공")
except Exception as e:
    print(f"[ERROR] CAN 버스 초기화 실패: {e}")
    exit(1)

# 2. ODrive 초기화
TOLERANCE = 2
encoder_ppr = 8192

reduction_ratio_0 = -46.003
reduction_ratio_1 = -46.003
reduction_ratio_2 = 28.4998
reduction_ratio_3 = 5.5
reduction_ratio_4 = -5
reduction_ratio_5 = 5

# 모든 노드에 Position Control 모드 및 Closed Loop 상태 설정
for node in NodeID:
    send_position_control_mode(bus, node)
    send_closed_loop_state(bus, node)

