# 4次のルンゲクッタで積分をするだけ
import numpy as np

def normalize_quaternion(q):
    return q / np.linalg.norm(q)

def quat_mult(q1, q2):
    """クォータニオンの積を計算"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

def quaternion_derivative(q, omega):
    """クォータニオンの微分を計算"""
    omega_q = np.hstack(([0], omega))  # (0, wx, wy, wz)
    q_dot = 0.5 * quat_mult(q, omega_q)
    return q_dot

def integrate_gyro_euler(q, omega, dt):
    """オイラー法でジャイロデータを積分"""
    q_dot = quaternion_derivative(q, omega)
    q_new = q + q_dot * dt
    return normalize_quaternion(q_new)

# 初期クォータニオン（単位クォータニオン）
q = np.array([1, 0, 0, 0])
dt = 0.01  # 時間刻み (10ms)
gyro_data = np.array([0.1, -0.2, 0.05])  # 角速度 (rad/s)

# クォータニオンを更新
q = integrate_gyro_euler(q, gyro_data, dt)
print(q)
