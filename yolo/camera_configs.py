import cv2
import numpy as np

# 效果好
left_camera_matrix = np.array([[665.968358, 0.241530, 634.973987], [0, 666.722271, 360.393886], [0, 0, 1]])

# left_distortion = np.array([[-0.154511565,0.325173292, 0.006934081,0.017466934, -0.340007548]])
left_distortion = np.array([[-0.051181929526, -0.0339436888389, 0.00021644011746, 0.0019906243918, 0.0019906243918]])

right_camera_matrix = np.array([[671.645311, -0.198615, 626.733130], [0, 672.033103, 377.909841], [0, 0, 1]])

# right_distortion = np.array([[-0.192887524,0.706728768, 0.004233541,0.021340116,-1.175486913]])
right_distortion = np.array([[-0.050092214685, -0.041507914265, 0.00009502001542, 0.0011311122168, 0.033998734668]])

R = np.array([[0.999997697, 0.000809551, 0.001987619],
              [-0.000797875, 0.999982463, -0.005868396],
              [-0.001992335, 0.005866796, 0.999980805]])

T = np.array([-60.94770179, 0.048695591, -0.217858121])


size = (1280, 720)  # open windows size
# R1:左摄像机旋转矩阵, P1:左摄像机投影矩阵, Q:重投影矩阵
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R, T)

# 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

# print(Q[2][3])

# import numpy as np


# 仅仅是一个示例


# 双目相机参数
class StereoCamera(object):
    def __init__(self):
        # 左相机内参
        self.cam_matrix_left = left_camera_matrix
        # 右相机内参
        self.cam_matrix_right = right_camera_matrix

        # 左右相机畸变系数:[k1, k2, p1, p2, k3]
        self.distortion_l = left_distortion
        self.distortion_r = right_distortion

        # 旋转矩阵
        self.R = R

        # 平移矩阵
        self.T = T

        # 焦距
        self.focal_length = 669.3776869999999  # 默认值，一般取立体校正后的重投影矩阵Q中的 Q[2,3]

        # 基线距离
        self.baseline = 60.94770179  # 单位：mm， 为平移向量的第一个参数（取绝对值）
