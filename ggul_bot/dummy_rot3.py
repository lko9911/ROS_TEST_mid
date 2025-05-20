import sys
import time
import odrive
from odrive.enums import *
import math
import can
import struct
from enum import IntEnum

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

    # 응답 대기
    expected_reply_id = generate_can_id(node_id, CommandID.Get_Encoder_Estimates)
    start_time = time.time()
    while time.time() - start_time < 0.1:  # 50ms까지 대기
        rx_msg = bus.recv(timeout= 0.1)
        if rx_msg is None:
            continue
        if rx_msg.arbitration_id != expected_reply_id:
            continue  # 다른 노드의 응답이면 무시
        if len(rx_msg.data) < 4:
            continue

        pos_estimate = struct.unpack('<f', rx_msg.data[0:4])[0]
        return pos_estimate

    raise TimeoutError(f"[에러] Node {node_id} 응답 없음 (ID=0x{expected_reply_id:X})")


#  odrv0.axis0.controller.input_pos 함수(CAN.ver)
def send_input_pos(bus: can.Bus, node_id: int, input_pos: float, vel_ff: float = 0.0):
    INPUT_POS_CMD = CommandID.Set_Input_Pos
    can_id = generate_can_id(node_id, INPUT_POS_CMD)

    payload = generate_can_message(input_pos, vel_ff)

    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"[전송] Node {node_id} → 목표 위치 {input_pos:.2f} 전송 완료")
    except can.CanError as e:
        print(f"[에러] Node {node_id} 위치 전송 실패: {e}")
        raise

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
            print(f"[경고] Node {node_id} 위치 확인 실패 (무시됨): {e}")
            all_reached = False
    return all_reached

# Position - trajectory 제어 모드 설정 함수 (control_mode=3, input_mode=16)
def send_position_control_mode(bus: can.Bus, node_id: int):
    can_id = generate_can_id(node_id, CommandID.Set_Controller_Mode)
    payload = b'\x03\x00\x00\x00' + b'\x05\x00\x00\x00'  # 3 = POSITION_CONTROL, 1 = PASSTHROUGH
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
        return [max_vel_limit] * 6, [max_accel_limit] * 6  # 모두 0 이동 시에도 최소값 유지

    speed_limits = []
    accel_limits = []

    for distance in distances:
        ratio = distance / max_distance
        vel = max((ratio * max_vel_limit), 1.0)      # 최소 속도 제한 (예: 3.0)
        accel = max((ratio * max_accel_limit), 1.0)  # 최소 가속도 제한 (예: 3.0)
        speed_limits.append(vel)
        accel_limits.append(accel)

    return speed_limits, accel_limits

# ================================================
# 조인트 세트 처리 함수
# ================================================
def process_joint_set(joint_angles):
    print(f"\nProcessing joint angles: {joint_angles}")

    joint_angles = [math.degrees(angle) for angle in joint_angles]
    print(f"Converted joint angles to degrees: {joint_angles}")
    
    if len(joint_angles) < 6:
        print('Not enough joint angles received.')
        return

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

    # 속도/가속도 설정 계산
    speed_limits, accel_limits = calculate_trap_traj_speeds(
        current_positions, target_positions_encoder, max_vel_limit, max_accel_limit
    )

    # 각 노드에 속도/가속 설정 전송
    for i, node_id in enumerate(NodeID):
        send_traj_vel_limit(bus, node_id, speed_limits[i])
        send_traj_accel_limits(bus, node_id, accel_limits[i], accel_limits[i])

    # 목표 위치 명령 한 번에 전송
    for i, node_id in enumerate(NodeID):
        try:
            print(f"[명령] Node {node_id} → 목표 위치: {target_positions[i]:.2f}")
            send_input_pos(bus, node_id, target_positions[i])
        except Exception as e:
            print(f"[오류] Node {node_id} 위치 명령 실패: {e}")

    print("Motors are moving...")

    retry_count = 0
    while not motors_reached_target(bus, target_positions, TOLERANCE):
        time.sleep(0.01)
        retry_count += 1
        if retry_count > 200:
            print("[경고] 반복 초과. 일부 노드가 도달하지 않았을 수 있습니다.")
            break

    print("Motors have reached their target positions.")

# ================================================
# ODrive 설정 관련 상수 및 함수
# ================================================
CHANNEL = 'COM3'
BITRATE = 250000

try:
    bus = can.interface.Bus(interface='slcan', channel=CHANNEL, bitrate=BITRATE) # CAN BUS 연결 확인
    print("[INFO] CAN 버스 연결 성공")
except Exception as e:
    print(f"[ERROR] CAN 버스 초기화 실패: {e}")
    sys.exit(1)


TOLERANCE = 1
encoder_ppr = 8192

# 감속비
reduction_ratio_0 = -46.003
reduction_ratio_1 = -46.003
reduction_ratio_2 = 28.4998
reduction_ratio_3 = 5.5
reduction_ratio_4 = -5
reduction_ratio_5 = 5


# 포지션 제어 모드로 설정
for node in NodeID:
    send_position_control_mode(bus, node)


process_joint_set([-2.5370661093203832, -0.6379914215368568, -1.6878682121504125, 0.8157341694464534, 2.1759268824437124, 3.1415907833656505])
time.sleep(1)
process_joint_set([-3.142, 0.873, -2.094, -1.222, -1.5708, 0.0])
time.sleep(5)
process_joint_set([0,0,0,0,0,0])
time.sleep(5)
# Closed loop 컨트롤 제어 모드로 설정
#for node in NodeID:
    #send_closed_loop_state(bus, node) 
    
# ================================================
    # ===========================
    # 직접 조인트 각도 입력 (라디안 단위)
    # 예: [30도, 60도, 45도, 90도, 0도, -30도] → 라디안으로 변환
    # ===========================
'''
joint_angles = [-3.142, 0.873, -2.094, -1.222, -1.5708, 0.0] 
#joint_angles = [-2.788, -0.5, -1.690, 0.951, 1.935, -3.1412] 
  
#joint_angles = [0,0,0,0,0,0]

print("=== ROS 없이 직접 조인트 명령 실행 ===")
print(f"입력한 조인트 각도 (라디안): {joint_angles}")
try:
    process_joint_set(joint_angles)
    time.sleep(5)
except Exception as e:
    print(f"[에러] 조인트 세트 실행 중 예외 발생: {e}")
    '''
