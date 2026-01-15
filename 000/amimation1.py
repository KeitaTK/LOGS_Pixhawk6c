# UDPでデータをMatlabに送信する

import time
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

# ここからUDP通信の設定
# UDP通信の設定
UDP_IP = "192.168.11.46"  # Windows PCのIPアドレスを指定
UDP_PORT = 5003  # ポート番号を指定 (Windows側と合わせる)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)




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

    print("Rotation Vector Quaternion:")
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
    print(
        "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    )

    print("送信開始")

    # データをUDPパケットに格納し送信
    message = f"{quat_i},{quat_j},{quat_k},{quat_real}"
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

    print("")
