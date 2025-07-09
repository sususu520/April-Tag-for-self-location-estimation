# Modbus_Server.py
from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.datastore.store import ModbusSequentialDataBlock
import numpy as np
import cv2
import pupil_apriltags as apriltag
import time
from threading import Thread

# カメラ内部パラメータ（単位：ピクセル）
camera_matrix = np.array([[1428.6, 0, 647.95],
                          [0, 1420.1, 328.6],
                          [0, 0, 1]])
dist_coeffs = np.array([[-0.0398, 2.522, 0.0011, 0.0017, -11.4147]])
tag_size = 0.162
known_tags = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([0.4, 0.0, 0.0]),
    2: np.array([0.8, 0.0, 0.0]),
    3: np.array([1.2, 0.0, 0.0])
}


# 負の値をModbus対応形式に変換 (2の補数)
def to_modbus_int(val):
    return val & 0xFFFF


# Modbus データ領域の初期化
store = ModbusSlaveContext(
    di=ModbusSequentialDataBlock(0, [0] * 100),
    co=ModbusSequentialDataBlock(0, [0] * 100),
    hr=ModbusSequentialDataBlock(0, [0] * 100),
    ir=ModbusSequentialDataBlock(0, [0] * 100)
)
context = ModbusServerContext(slaves=store, single=True)


# エラー出力
def send_error(error_type, code, message):
    print(f"❌ [{error_type}] {code}: {message}")


# カメラ姿勢取得と Modbus 書き込み
def send_pose():
    detector = apriltag.Detector(families='tag36h11')
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        send_error("カメラ", "0x02-0x02", "カメラ起動失敗")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            send_error("カメラ", "0x02-0x03", "フレーム取得失敗")
            time.sleep(0.1)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(camera_matrix[0, 0], camera_matrix[1, 1],
                           camera_matrix[0, 2], camera_matrix[1, 2]),
            tag_size=tag_size
        )

        print(f"検出タグ: {[tag.tag_id for tag in tags]}")

        positions = []
        yaws = []
        for tag in tags:
            if tag.tag_id not in known_tags:
                continue

            R = tag.pose_R
            t = tag.pose_t.flatten()
            cam_pos_world = known_tags[tag.tag_id] + R.T @ (-t)

            cam_pos = np.array([
                cam_pos_world[0],  # X
                -cam_pos_world[2],  # Z→Y
                -cam_pos_world[1]  # Y→Z
            ])

            adjusted_R = np.array([R[:, 0], R[:, 2], R[:, 1]]).T
            yaw = np.arctan2(adjusted_R[1, 0], adjusted_R[0, 0])

            positions.append(cam_pos)
            yaws.append(yaw)

        if positions:
            avg_pos = np.mean(positions, axis=0)
            avg_yaw = np.mean(yaws)

            # 単位変換: m → mm, rad → deg
            x_mm = int(avg_pos[0] * 10)
            y_mm = int(avg_pos[1] * 10)
            z_mm = int(avg_pos[2] * 10)
            yaw_d = int(np.degrees(avg_yaw))

            # デバッグ出力
            print(
                f"平均姿勢: x={avg_pos[0]:.3f}m, y={avg_pos[1]:.3f}m, z={avg_pos[2]:.3f}m, yaw={np.degrees(avg_yaw):.1f}°")
            print(f"→ 整数変換: x={x_mm}mm, y={y_mm}mm, z={z_mm}mm, yaw={yaw_d}°")

            # 整数値をレジスタに変換（負の値は2の補数形式で）
            regs = [
                to_modbus_int(x_mm),
                to_modbus_int(y_mm),
                to_modbus_int(z_mm),
                0,
                0,
                0,
                0,
                0,
                to_modbus_int(yaw_d)
            ]

            # Holding Register (アドレス0から4つ) に書き込み
            context[0].setValues(4, 3, regs)

        time.sleep(0.2)


# サービス起動（ポート1502）
if __name__ == "__main__":
    Thread(target=send_pose, daemon=True).start()
    print("Modbusサーバー起動: 0.0.0.0:1502")
    print("クライアントは保持レジスタ(FC03)のアドレス40001から4レジスタを読み取ってください")
    StartTcpServer(context, address=("0.0.0.0", 1502))