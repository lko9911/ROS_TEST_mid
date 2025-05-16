import numpy as np

def euler_to_dcm_deg(roll_deg, pitch_deg, yaw_deg): 
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    return Rz @ Ry @ Rx

R = euler_to_dcm_deg(0, 90, 0)

# 예시 입력 좌표
P_cam = np.array([0.0, 0.0, -0.5])
P_base = R @ P_cam

# P_cam = np.array([0.5, -0.1, 0.2])
print("R:\n", R)
print("입력 P_cam:", P_cam)
print("변환 후 P_base:", P_base)
