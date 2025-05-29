import numpy as np
import cv2
import pupil_apriltags as apriltag
import matplotlib.pyplot as plt
import time

# カメラの内部パラメータ（単位：ピクセル）
camera_matrix = np.array([[1428.6, 0, 647.95],
[0, 1420.1, 328.6],
[0, 0, 1]])
dist_coeffs = np.array([[-0.0398, 2.522, 0.0011, 0.0017, -11.4147]])

# 既知のタグの位置（単位：メートル）※座標系に注意
tag_size = 0.162
known_tags = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([0.4, 0.0, 0.0]),
    2: np.array([0.8, 0.0, 0.0]),
    3: np.array([1.2, 0.0, 0.0])
}

# 地図の初期化
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(0, 2)
ax.set_ylim(0, 2)
ax.set_xlabel('X axis (m)')
ax.set_ylabel('Y axis (m)')
ax.set_title('Camera Position and Yaw Direction')
ax.legend()

# AprilTag 検出器
detector = apriltag.Detector(families='tag36h11')

# カメラの起動
cap = cv2.VideoCapture(0)
prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
     break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray, estimate_tag_pose=True,camera_params=(camera_matrix[0,0], camera_matrix[1,1],
    camera_matrix[0,2], camera_matrix[1,2]),tag_size=tag_size)

    positions = []
    yaws = []

    for tag in tags:
        if tag.tag_id not in known_tags:
            continue

        # 重要処理：座標軸の変換
        R = tag.pose_R
        t = tag.pose_t.flatten()

        # カメラ座標系からワールド座標系へ変換（AprilTagの出力がカメラ→タグの変換であることを前提）
        # Y軸とZ軸の入れ替え＋符号反転に注意
        cam_pos_world = known_tags[tag.tag_id] + R.T @ (-t)

        # Y軸とZ軸を入れ替えてZ軸を反転（検証済の調整）
        cam_pos = np.array([
        cam_pos_world[0], # X軸はそのまま
        -cam_pos_world[2], # 元のZ軸を反転してY軸として使用
        -cam_pos_world[1] # 元のY軸をZ軸に使用（符号反転）
        ])

        # 姿勢行列の軸並べ替え
        adjusted_R = np.array([R[:,0], R[:,2], R[:,1]]).T

        # Yaw角（Z軸回りの回転）を算出
        yaw = np.arctan2(adjusted_R[1,0], adjusted_R[0,0])

        positions.append(cam_pos)
        yaws.append(yaw)

        # 検出されたタグの枠を描画
        cv2.polylines(frame, [np.int32(tag.corners)], True, (0,255,0), 2)

    if positions:
        positions = np.array(positions)
        yaws = np.array(yaws)
        avg_pos = np.mean(positions, axis=0)
        avg_yaw = np.mean(yaws)

        # 地図の更新（X座標と変換後のY座標を使用）
        ax.clear()
        ax.set_xlim(0, 2)
        ax.set_ylim(0, 2)
        ax.plot(avg_pos[0], avg_pos[1], 'ro', label='Camera Position')

        # カメラの向きを矢印で描画（Z軸方向を利用）
        z_axis_cam = R[:, 2]
        dx = 0.2 * z_axis_cam[0]
        dy = -0.2 * z_axis_cam[2]
        ax.arrow(avg_pos[0], avg_pos[1], dx, dy,head_width=0.03, head_length=0.03,fc='blue', ec='blue', label='Camera Direction')

        ax.legend()
        plt.draw()
        plt.pause(0.001)

        # 情報の表示（FPSとカメラの3D位置）
        fps = 1.0 / (time.time() - prev_time)
        prev_time = time.time()
        text = (f'FPS: {fps:.2f}\n'
                f'X: {avg_pos[0]*1000:.0f} mm\n'
                f'Y: {avg_pos[1]*1000:.0f} mm\n' # 元のZ軸
                f'Z: {avg_pos[2]*1000:.0f} mm') # 元のY軸

        for i, line in enumerate(text.split('\n')):
            cv2.putText(frame, line, (10, 30 + i*20),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

    # 結果表示
    cv2.imshow('AprilTag Detection', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
