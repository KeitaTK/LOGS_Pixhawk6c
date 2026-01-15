# 4次のルンゲクッタで積分をするだけ
# シグナルハンドラで定周期実行
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
import signal
import math
from multiprocessing import shared_memory



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

def rm_memory():
    shm_name = ["q_log_memory",]
    for i in range(1):
        # 共有メモリを開く (存在しない場合はエラーとなる)
        try:
            shm = shared_memory.SharedMemory(name=shm_name[i])
        except FileNotFoundError:
            print(f"'{shm_name[i]}'正常に削除済み")
        else:
            # 共有メモリを削除
            shm.close()
            shm.unlink()
            print(f"共有メモリ '{shm_name[i]}' を削除しました")
    print("共有メモリ解放完了")

def scheduler(arg1, args2):
     # ジャイロセンサを取得
    gyro_x, gyro_y, gyro_z = bno.gyro

    dt = 0.01  # 時間刻み (10ms)
    gyro_data = np.array([gyro_x, gyro_y, gyro_z])  # 角速度 (rad/s)

    q_log = q_log_memory[:]
    print("q_log",q_log)
    q = integrate_gyro_euler(q_log, gyro_data, dt)
    print(q)
    
    # メモリに格納
    q_log_memory[:] = q

    print("送信開始")

    # データをUDPパケットに格納し送信
    message = f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}"
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))


if __name__ == '__main__':

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    t_pre = 0
    q_ref = None  # 初期クォータニオン


    rm_memory()
    # 共有メモリを作成
    SHAPE_q_log = (4,)
    size_q_log = math.prod(SHAPE_q_log)*np.dtype("float").itemsize
    shm_q_log = shared_memory.SharedMemory(create=True, size=size_q_log, name="q_log_memory")
    q_log_memory = np.ndarray(shape=SHAPE_q_log, dtype=float, buffer=shm_q_log.buf)
    q_log_memory[:] = [1, 0, 0, 0]
    print(q_log_memory)


 
    # ここからUDP通信の設定
    # UDP通信の設定
    UDP_IP = "192.168.11.3"  # Windows PCのIPアドレスを指定
    UDP_PORT = 5003  # ポート番号を指定 (Windows側と合わせる)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    signal.signal(signal.SIGALRM, scheduler)
    signal.setitimer(signal.ITIMER_REAL, 0.01, 0.01)

    time.sleep(1000)
