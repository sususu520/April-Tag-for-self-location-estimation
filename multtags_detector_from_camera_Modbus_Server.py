from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.datastore.store import ModbusSequentialDataBlock
import numpy as np
import cv2
import pupil_apriltags as apriltag
import struct
import time

# 相机内参
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

# Modbus 数据区初始化
store = ModbusSlaveContext(
    di=ModbusSequentialDataBlock(0, [0]*100),
    co=ModbusSequentialDataBlock(0, [0]*100),
    hr=ModbusSequentialDataBlock(0, [0]*100),
    ir=ModbusSequentialDataBlock(0, [0]*100)
)
context = ModbusServerContext(slaves=store, single=True)

# Float编码为两个16位寄存器
def float_to_regs(val):
    b = struct.pack('>f', float(val))
    return [int.from_bytes(b[:2], 'big'), int.from_bytes(b[2:], 'big')]

def send_error(error_type, code, message):
    print(f"❌ [{error_type}] {code}: {message}")

def send_pose():
    detector = apriltag.Detector(families='tag36h11')
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        send_error("相機", "0x02-0x02", "カメラ起動失敗")
        return

    prev_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            send_error("相機", "0x02-0x03", "フレーム取得失敗")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(camera_matrix[0,0], camera_matrix[1,1],
                           camera_matrix[0,2], camera_matrix[1,2]),
            tag_size=tag_size
        )

        print(f"Detected tags: {[tag.tag_id for tag in tags]}")

        positions = []
        yaws = []
        for tag in tags:
            if tag.tag_id not in known_tags:
                continue

            R = tag.pose_R
            t = tag.pose_t.flatten()
            cam_pos_world = known_tags[tag.tag_id] + R.T @ (-t)

            cam_pos = np.array([
                cam_pos_world[0],        # X
                -cam_pos_world[2],       # Z 变 Y
                -cam_pos_world[1]        # Y 变 Z
            ])

            adjusted_R = np.array([R[:,0], R[:,2], R[:,1]]).T
            yaw = np.arctan2(adjusted_R[1,0], adjusted_R[0,0])

            positions.append(cam_pos)
            yaws.append(yaw)

        if positions:
            avg_pos = np.mean(positions, axis=0)
            avg_yaw = np.mean(yaws)
            print(f"平均姿勢：x={avg_pos[0]:.3f}, y={avg_pos[1]:.3f}, z={avg_pos[2]:.3f}, yaw={np.degrees(avg_yaw):.1f}°")

            # 打包 float 到寄存器（共 6 个寄存器：x,y,z）
            regs = (
                float_to_regs(avg_pos[0]) +
                float_to_regs(avg_pos[1]) +
                float_to_regs(avg_pos[2])
            )
            context[0].setValues(3, 0, regs)

        time.sleep(0.2)

# 启动服务（端口1502）
if __name__ == "__main__":
    from threading import Thread
    Thread(target=send_pose).start()
    StartTcpServer(context, address=("0.0.0.0", 1502))