# オイラー方で積分をするだけ
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
    q = np.array([1, 0, 0, 0])
    print(q)
    interval_gyro = 0.01

    while True:
        next_time_gyro =  round((time.perf_counter() + interval_gyro),6) # 次回実行時刻を取得

        # ジャイロセンサを取得
        gyro_x, gyro_y, gyro_z = bno.gyro

        dt = interval_gyro  # 時間刻み (10ms)
        gyro_data = np.array([gyro_x, gyro_y, gyro_z])  # 角速度 (rad/s)

        q = integrate_gyro_euler(q, gyro_data, dt)
        # print(q)

        print("送信開始")

        # データをUDPパケットに格納し送信
        message = f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

        sleep_time_gyro = next_time_gyro - time.perf_counter()
        if sleep_time_gyro > 0:
            time.sleep(sleep_time_gyro) # 次回実行時刻まで待つ
            print(0.01 - sleep_time_gyro)

