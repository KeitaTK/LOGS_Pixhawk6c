# 4次のルンゲクッタで積分
# 加速度センサで長期のずれを補正する
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

import socket
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

def normalize_vector(v):
    """ベクトルを正規化"""
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-6 else np.array([0, 0, 1])

def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    q = q / norm if norm > 1e-6 else np.array([1, 0, 0, 0])
    
    # w が負なら反転
    if q[0] < 0:
        q = -q
    return q


def quaternion_conjugate(q):
    """クォータニオンの共役を計算"""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])
    

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

def correction_quaternion(a_prime, a0, alpha=0.01):
    """補正クォータニオンを計算"""
    n_a = np.cross(a_prime, a0)
    n_a = normalize_vector(n_a)

    theta_a = np.arccos(np.clip(np.dot(a_prime, a0), -1.0, 1.0))
    theta_a *= alpha

    q_a = np.hstack([
        np.cos(theta_a / 2),
        np.sin(theta_a / 2) * n_a
    ])
    return normalize_quaternion(q_a)

def quaternion_derivative(q, omega):
    """クォータニオンの微分を計算"""
    omega_q = np.hstack(([0], omega))  # (0, wx, wy, wz)
    q_dot = 0.5 * quat_mult(q, omega_q)
    return q_dot

def integrate_gyro_rk4(q, omega, dt):
    """4次のルンゲクッタ法でジャイロデータを積分"""
    k1 = quaternion_derivative(q, omega) * dt
    k2 = quaternion_derivative(q + k1 / 2, omega) * dt
    k3 = quaternion_derivative(q + k2 / 2, omega) * dt
    k4 = quaternion_derivative(q + k3, omega) * dt
    q_new = q + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return normalize_quaternion(q_new)

def integrate_gyro_euler(q, omega, dt):
    """オイラー法でジャイロデータを積分"""
    q_dot = quaternion_derivative(q, omega)
    q_new = q + q_dot * dt
    return normalize_quaternion(q_new)

if __name__ == '__main__':

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    t_pre = 0
    q_ref = None  # 初期クォータニオン

 
    # ここからUDP通信の設定
    # UDP通信の設定
    UDP_IP = "192.168.11.3"  # Windows PCのIPアドレスを指定
    UDP_PORT = 5003  # ポート番号を指定 (Windows側と合わせる)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 初期クォータニオン（単位クォータニオン）
    q_w = np.array([1, 0, 0, 0])
    interval_gyro = 0.014
    v_ref = np.array([0, 0, 1])

    while True:
        next_time_gyro =  round((time.perf_counter() + interval_gyro),6) # 次回実行時刻を取得

        # ジャイロセンサを取得しクオータニオンに変換
        gyro_data = np.array(bno.gyro)  # 角速度 (rad/s)
        # q_w = integrate_gyro_rk4(q_w, gyro_data, interval_gyro)
        q_w = integrate_gyro_euler(q_w, gyro_data, interval_gyro)
    
        q_w_inv = quaternion_conjugate(q_w)
        # print(q_w_inv)

        # 加速度センサを取得し重力方向を計算
        accel_data = np.array(bno.acceleration)  # 角速度 (rad/s)
        accel_q = np.hstack(([0], accel_data))

        # 加速度センサデータを慣性座標系へ変換
        accel_q_down = quat_mult(q_w, quat_mult(accel_q, q_w_inv)) # クオータニオンの形
        accel_down = accel_q_down[1:4]
        
        # 重力方向を正規化
        g_est = normalize_vector(accel_q_down[1:4])


        # 補正クォータニオン計算
        q_correction = correction_quaternion(g_est, v_ref)
        # print(q_correction)

        q_w = normalize_quaternion(quat_mult(q_w,q_correction))
        # print("補正後クォータニオン:", q_w)


        # print("送信開始")

        # データをUDPパケットに格納し送信
        message = f"{q_w[0]:.6f},{q_w[1]:.6f},{q_w[2]:.6f},{q_w[3]:.6f}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

        sleep_time_gyro = next_time_gyro - time.perf_counter()
        if sleep_time_gyro > 0:
            time.sleep(sleep_time_gyro) # 次回実行時刻まで待つ
            # print(0.01 - sleep_time_gyro)
        else:
            print("9999999999999999999999999999999999999999999999999")

