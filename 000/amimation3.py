# UDPでデータをMatlabに送信する

import time
import numpy as np
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

# i2c = busio.I2C(board.GP5, board.GP4, frequency=400000)
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
UDP_IP = "192.168.11.2"  # Windows PCのIPアドレスを指定
UDP_PORT = 5003  # ポート番号を指定 (Windows側と合わせる)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for i in range(100):
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member




while True:
    # print(time.monotonic())
    # print("Acceleration:")
    # accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    # print("")

    # print("Gyro:")
    # gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    # print("")

    # print("Magnetometer:")
    # mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    # print("")

    # print("Rotation Vector Quaternion:")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
    # print(
    #     "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    # )

    # 初期位置から見た姿勢を求める
    quat = np.array([quat_real, quat_i, quat_j, quat_k])  # scipyは[w, x, y, z]の順

    # クォータニオンがゼロノルムでないことを確認
    norm = np.linalg.norm(quat)
    if norm < 1e-6:  # ゼロノルムのクォータニオンを無視
        print("Received invalid quaternion, retrying...")
        continue

    if q_ref is None:
        q_ref = R.from_quat(quat)  # 最初のクォータニオンを基準として保存

    q_corrected = q_ref.inv() * R.from_quat(quat)


    # 補正後のクォータニオンを取得
    corrected_quat = q_corrected.as_quat()  # [x, y, z, w]


    print("送信開始")

    # データをUDPパケットに格納し送信
    message = f"{corrected_quat[0]:.6f},{corrected_quat[1]:.6f},{corrected_quat[2]:.6f},{corrected_quat[3]:.6f}"
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

    print(corrected_quat)
