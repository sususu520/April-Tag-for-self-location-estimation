import cv2
import numpy as np
import glob
#チェスボードのサイズを設定（行と列の数）                                                       设置棋盘格的尺寸 (行数和列数)
chessboard_size = (8, 6)
# チェスボードの3Dポイントを準備します。例えば、(0, 0, 0), (1, 0, 0), (2, 0, 0), ..., 9x6 チェスボードを計算します。
# 准备棋盘的3D点，如 (0, 0, 0), (1, 0, 0), (2, 0, 0), ..., 计算 9x6 格的棋盘
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
# すべての3Dポイントと2Dポイントを保存                                                          保存所有的3D点和2D点
objpoints = [] # 実世界の座標での3Dポイント                                                    3D点在现实世界中的坐标
imgpoints = []  #  画像内の2Dポイント                                                        2D点在图像中的坐标
# チェスボードの画像ファイルを読み取る                                                           读取棋盘图案的图片文件
images = glob.glob('logic_camera_calibration\*.jpg')  # 実際の画像保存パスに置き換える                替换成你实际存储图片的路径
for fname in images:
   img = cv2.imread(fname)
   gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   # チェスボードのコーナーを見つける                                                           找到棋盘格的角点
   ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
   # コーナーが見つかったら、それらを保存する                                                     如果找到角点，保存它们
   if ret:
       objpoints.append(objp)
       imgpoints.append(corners)
       # コーナーを描画して表示                                                               画出角点并显示
       img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
       cv2.imshow('img', img)
       cv2.waitKey(500)
cv2.destroyAllWindows()
# カメラの較正を行い、内部パラメータ行列を計算する                                                 进行相机标定，计算内参矩阵
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# カメラの内部パラメータ行列と歪み係数を出力する                                                   输出相机内参矩阵和畸变系数
print("Camera matrix:")
print(camera_matrix)
print("Distortion coefficients:")
print(dist_coeffs)