# Robot_Operation.py
import time
import odrive
from odrive.enums import *
import math
import json

# ---- 설정 ----
TOLERANCE = 2
encoder_ppr = 8192
reduction_ratios = [46.003, 46.003, 28.4998, 30, 5.5, 5]

ODRIVE0_SERIAL = "3460354E3033"
ODRIVE1_SERIAL = "345135523033"
ODRIVE2_SERIAL = "345A354E3033"

print("Finding ODrives...")
odrv0 = odrive.find_any(serial_number=ODRIVE0_SERIAL)
odrv1 = odrive.find_any(serial_number=ODRIVE1_SERIAL)
odrv2 = odrive.find_any(serial_number=ODRIVE2_SERIAL)
print("All ODrives connected!")

# ---- 초기 세팅 ----
for odrv in [odrv0, odrv1, odrv2]:
    odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# ---- 유틸 함수 ----
def angle_to_encoder_counts(angle, reduction_ratio):
    mechanical_revs = angle / 360.0
    return mechanical_revs * reduction_ratio * encoder_ppr

def motors_reached_target(positions):
    return (
        abs(odrv0.axis0.encoder.pos_estimate - positions[0]) < TOLERANCE and
        abs(odrv0.axis1.encoder.pos_estimate - positions[1]) < TOLERANCE and
        abs(odrv1.axis0.encoder.pos_estimate - positions[2]) < TOLERANCE and
        abs(odrv1.axis1.encoder.pos_estimate - positions[3]) < TOLERANCE and
        abs(odrv2.axis0.encoder.pos_estimate - positions[4]) < TOLERANCE and
        abs(odrv2.axis1.encoder.pos_estimate - positions[5]) < TOLERANCE
    )

def calculate_trap_traj_speeds(current_positions, target_positions_encoder, max_vel_limit, max_accel_limit):
    distances = [abs(target - current) for target, current in zip(target_positions_encoder, current_positions)]
    max_distance = max(distances)
    if max_distance == 0:
        return [0] * 6, [0] * 6
    speed_limits = [(d / max_distance) * max_vel_limit for d in distances]
    accel_limits = [(d / max_distance) * max_accel_limit for d in distances]
    return speed_limits, accel_limits

# ---- 주 제어 함수 ----
def process_joint_angles(joint_angles_rad):
    joint_angles_deg = [math.degrees(a) for a in joint_angles_rad]
    print(f"[INFO] Received joint angles (deg): {joint_angles_deg}")

    target_positions = [
        (reduction_ratios[i] / 360.0) * joint_angles_deg[i] for i in range(6)
    ]
    target_positions_encoder = [
        angle_to_encoder_counts(joint_angles_deg[i], reduction_ratios[i]) for i in range(6)
    ]

    current_positions = [
        odrv0.axis0.encoder.pos_estimate,
        odrv0.axis1.encoder.pos_estimate,
        odrv1.axis0.encoder.pos_estimate,
        odrv1.axis1.encoder.pos_estimate,
        odrv2.axis0.encoder.pos_estimate,
        odrv2.axis1.encoder.pos_estimate,
    ]

    max_vel_limit = 7
    max_accel_limit = 6

    speed_limits, accel_limits = calculate_trap_traj_speeds(current_positions, target_positions_encoder, max_vel_limit, max_accel_limit)

    for axis, speed, accel in zip(
        [odrv0.axis0, odrv0.axis1, odrv1.axis0, odrv1.axis1, odrv2.axis0, odrv2.axis1],
        speed_limits, accel_limits):
        axis.trap_traj.config.vel_limit = speed
        axis.trap_traj.config.accel_limit = accel
        axis.trap_traj.config.decel_limit = accel

    odrv0.axis0.controller.input_pos = target_positions[0]
    odrv0.axis1.controller.input_pos = target_positions[1]
    odrv1.axis0.controller.input_pos = target_positions[2]
    odrv1.axis1.controller.input_pos = target_positions[3]
    odrv2.axis0.controller.input_pos = target_positions[4]
    odrv2.axis1.controller.input_pos = target_positions[5]

    print("Motors moving...")

    while not motors_reached_target(target_positions):
        time.sleep(0.001)

    print("Motors reached their target.")

def read_and_send_joint_values(log_path):
    with open(log_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            try:
                data = json.loads(line)
                joint_values = data.get("joint_values")
                if joint_values and len(joint_values) == 6:
                    print(f"[INFO] Line {line_num}: Sending joint values {joint_values}")
                    process_joint_angles(joint_values)
                    
                else:
                    print(f"[WARNING] Line {line_num}: Invalid or missing 'joint_values'")
            except json.JSONDecodeError as e:
                print(f"[ERROR] Line {line_num}: JSON decode error: {e}")
