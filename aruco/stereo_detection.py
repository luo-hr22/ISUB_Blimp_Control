import cv2
import camera_configs
import math
import time
import socket
import numpy as np


# 坐标转换，相机系->地系
def coordinate_transforming(location_c, phi, theta, psi, aruco_id):
    location_b = np.array([location_c[2], location_c[0], location_c[1]])
    c_theta = math.cos(theta)
    c_psi = math.cos(psi)
    c_phi = math.cos(phi)
    s_theta = math.sin(theta)
    s_psi = math.sin(psi)
    s_phi = math.sin(phi)
    transform_mat = np.array([[c_theta * c_psi, c_theta * s_psi, -s_theta],
                              [s_phi * s_theta * c_psi - c_phi * s_psi,
                               s_phi * s_theta * s_psi + c_phi * c_psi, s_phi * c_theta],
                              [c_phi * s_theta * c_psi + s_phi * s_psi,
                               c_phi * s_theta * s_psi - s_phi * c_psi, c_phi * c_theta]])
    inverse_transform_mat = np.linalg.inv(transform_mat)
    location_w = np.dot(inverse_transform_mat, location_b)
    location_w = - location_w + coordinate_w_aruco[aruco_id]
    return location_w


def solve(p1, p2, p3, p4, d1, d2, d3, d4):
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    p4 = np.array(p4)

    a = 2 * (p2 - p1)
    b = 2 * (p3 - p1)
    c = 2 * (p4 - p1)

    b1 = d1 * d1 - d2 * d2 + np.dot(p2, p2) - np.dot(p1, p1)
    b2 = d1 * d1 - d3 * d3 + np.dot(p3, p3) - np.dot(p1, p1)
    b3 = d1 * d1 - d4 * d4 + np.dot(p4, p4) - np.dot(p1, p1)

    matrix = np.array([a, b, c])
    constants = np.array([b1, b2, b3])

    position = np.linalg.solve(matrix, constants)
    return position


coordinate_w_aruco = np.array([[0, -0.102, -0.212], [0, 0.193, -0.212]])

coordinate_w_aruco4 = (0.235, 0, 0.212)
coordinate_w_aruco5 = (0, 0.242, 0.218)
coordinate_w_aruco6 = (0.235, 0, 0.586)
coordinate_w_aruco7 = (0, 0.260, 0.600)

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

ip_address = input("please input IP address（example：172.20.10.3）：")
# 设置目标地址和端口
host = ip_address  # 替换为你的树莓派的IP地址
port = 8888  # 必须与服务器端一致
# 连接到服务器
client_socket.connect((host, port))

video_path = f"http://{ip_address}:8080/video_feed1"
# video_path = 0
cap = cv2.VideoCapture(video_path)  # 打开并设置摄像头
cap.set(3, 2560)  # 宽度
cap.set(4, 720)  # 高度


'''def onmouse_pick_points(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # 单击左键
        three_d = param
        print('\n像素坐标 x = %d, y = %d' % (x, y))
        y_b = three_d[y][x][0]
        z_b = three_d[y][x][1]
        x = max(40, x)
        x = min(1240, x)
        y = max(40, y)
        y = min(680, y)
        count = 0
        x_b_sum = 0
        for x_p in range(x - 40, x + 40):
            for y_p in range(y - 40, y + 40):
                x_b = three_d[y_p][x_p][2]
                if x_b < 10:
                    count += 1
                    x_b_sum += x_b
        if count > 0:
            x_b = x_b_sum / count
            send_message = '{:.2f},{:.2f},{:.2f}'.format(x_b, y_b, z_b)
            client_socket.sendall(send_message.encode('utf-8'))
            print('鼠标回调已执行，发送', send_message)
        else:
            client_socket.sendall('无效点击，请重试'.encode('utf-8'))'''


def onmouse_pick_points(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        three_d = param
        print('\n像素坐标 x = %d, y = %d' % (x, y))
        location_p = np.array([x, y, 1])
        transform_mat = camera_configs.left_camera_matrix
        inverse_transform_mat = np.linalg.inv(transform_mat)
        location_i = np.dot(inverse_transform_mat, location_p)
        x_c = location_i[0] / 2.4
        y_c = location_i[1] / 2.4
        z_c = 1
        location_c = np.array([x_c, y_c, z_c])
        dis_unit = math.sqrt(x_c ** 2 + y_c ** 2 + z_c ** 2)
        x = max(40, x)
        x = min(1240, x)
        y = max(40, y)
        y = min(680, y)
        dis_sum = 0
        count = 0
        for x_p in range(x - 40, x + 40):
            for y_p in range(y - 40, y + 40):
                if three_d[y_p][x_p][2] < 10:
                    dis_sum += math.sqrt(three_d[y_p][x_p][0] ** 2 + three_d[y_p][x_p][1] ** 2 +
                                         three_d[y_p][x_p][2] ** 2)
                    count += 1
        if count > 0:
            dis = dis_sum / count
            location_c = location_c * (dis / dis_unit)
            x_b = location_c[2]
            y_b = location_c[0]
            z_b = location_c[1]
            send_message = ';2,{:.2f},{:.2f},{:.2f};'.format(x_b, y_b, z_b)
            client_socket.sendall(send_message.encode('utf-8'))
            print('鼠标回调已执行，发送', send_message)
        else:
            print('无效点击，请重试')


WIN_NAME = 'Deep disp'
cv2.namedWindow(WIN_NAME, cv2.WINDOW_AUTOSIZE)  # 窗口命名

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters()

start_time = time.time()
counter = 0
fps = cap.get(cv2.CAP_PROP_FPS)

try:
    while True:
        # empty_str = ''
        client_socket.sendall(b';0.00,0.00,0.00;')

        counter += 1

        # 接收响应
        # print('接收前')
        response_str = client_socket.recv(1024).decode('utf-8')
        # print(response_str)
        # print('接收后')
        if response_str == 'Hello':
            client_socket.sendall(b'Hello')
            continue
        '''elif response_str == '':
            continue'''
        response2 = response_str.split(';')
        response = response2[1].split(',')
        if response == ['']:
            continue
        # print(f"收到响应: {response}")
        if not response:
            continue
        yaw = float(response[0]) * math.pi / 180
        pitch = float(response[1]) * math.pi / 180
        roll = float(response[2]) * math.pi / 180
        '''yaw = 0
        pitch = 0
        roll = 0'''

        ret, frame = cap.read()
        frame1 = frame[0:720, 0:1280]
        frame2 = frame[0:720, 1280:2560]  # 割开双目图像

        imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)  # 将BGR格式转换成灰度图片
        imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # cv2.remap 重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
        # 依据MATLAB测量数据重建无畸变图片
        img1_rectified = cv2.remap(imgL, camera_configs.left_map1, camera_configs.left_map2, cv2.INTER_LINEAR)
        img2_rectified = cv2.remap(imgR, camera_configs.right_map1, camera_configs.right_map2, cv2.INTER_LINEAR)

        imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
        imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)

        # BM
        numberOfDisparities = 160

        stereo = cv2.StereoBM.create(numDisparities=16, blockSize=9)  # 立体匹配
        stereo.setROI1(camera_configs.validPixROI1)
        stereo.setROI2(camera_configs.validPixROI2)
        stereo.setPreFilterCap(31)
        stereo.setBlockSize(15)
        stereo.setMinDisparity(0)
        stereo.setNumDisparities(numberOfDisparities)
        stereo.setTextureThreshold(10)
        stereo.setUniquenessRatio(15)
        stereo.setSpeckleWindowSize(100)
        stereo.setSpeckleRange(32)
        stereo.setDisp12MaxDiff(1)

        disparity = stereo.compute(img1_rectified, img2_rectified)  # 计算视差

        disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                             dtype=cv2.CV_8U)  # 归一化函数算法

        threeD = cv2.reprojectImageTo3D(disparity, camera_configs.Q, handleMissingValues=True)  # 计算三维坐标数据值
        threeD = threeD * 16 / 1000

        # --------------------识别--------------------
        (corners, ids, rejected) = cv2.aruco.detectMarkers(imgL, arucoDict, parameters=arucoParams)
        # 验证*至少*一个 ArUco 标记被检测到
        corners_len = len(corners)
        if corners_len == 4:
            # 展平 ArUco ID 列表
            ids = ids.flatten()
            coordinate_w_sum = np.array([0, 0, 0])
            dis_lst = [0, 0, 0, 0]
            # 循环检测到的 ArUCo 角
            for (markerCorner, markerID) in zip(corners, ids):
                if int(markerID) > 1:
                    continue
                # 提取标记角（始终按左上角、右上角、右下角和左下角顺序返回）
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # 将每个 (x, y) 坐标对转换为整数
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                x1 = min(bottomLeft[0], topLeft[0])
                x2 = max(bottomRight[0], topRight[0])
                y1 = min(topRight[1], topLeft[1])
                y2 = max(bottomRight[1], bottomLeft[1])

                '''x1 = x1 + (x2 - x1) // 3
                y1 = y1 + (y2 - y1) // 3
                x2 = x2 - (x2 - x1) // 3
                y2 = y2 - (y2 - y1) // 3'''

                delta_x = (x2 - x1) / 11
                delta_y = (y2 - y1) / 11
                x_lst = [x1]
                y_lst = [y1]
                distance_lst = []
                x_sum = 0
                y_sum = 0
                xc_sum = 0
                yc_sum = 0
                zc_sum = 0
                distance_sum = 0
                for ii in range(0, 11):
                    x_lst.append(int(x1 + delta_x * (ii + 1)))
                for ii in range(0, 11):
                    y_lst.append(int(y1 + delta_y * (ii + 1)))
                    for jj in range(0, 11):
                        distance = math.sqrt(threeD[y_lst[ii]][x_lst[jj]][0] ** 2 +
                                             threeD[y_lst[ii]][x_lst[jj]][1] ** 2 +
                                             threeD[y_lst[ii]][x_lst[jj]][2] ** 2)
                        if distance > 10:
                            continue
                        distance_lst.append(distance)
                        x_sum += x_lst[jj]
                        y_sum += y_lst[ii]
                        distance_sum += distance
                        xc_sum += threeD[y_lst[ii]][x_lst[jj]][0]
                        yc_sum += threeD[y_lst[ii]][x_lst[jj]][1]
                        zc_sum += threeD[y_lst[ii]][x_lst[jj]][2]
                length = len(distance_lst)
                if length == 0:
                    '''distance = 160
                    x_aver = 0
                    y_aver = 0
                    coordinate_c = np.array([0, 0, 160])'''
                    corners_len -= 1
                    continue
                else:
                    distance = distance_sum / length
                    x_aver = x_sum / length
                    y_aver = y_sum / length
                    coordinate_c = np.array([xc_sum / length, yc_sum / length, zc_sum / length])
                coordinate_w = coordinate_transforming(coordinate_c, yaw, pitch, roll, int(markerID))

                coordinate_w_str = 'coordinate=({:.2f}, {:.2f}, {:.2f})'.format(coordinate_w[0], coordinate_w[1],
                                                                                coordinate_w[2])
                cv2.putText(frame1, coordinate_w_str, (bottomLeft[0], bottomLeft[1]), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 0, 255), 2)
                coordinate_w_sum = coordinate_w_sum + coordinate_w

                # 绘制有效点中心
                point_size = 1
                point_color = (0, 0, 255)  # BGR
                thickness = 4  # 可以为 0 、4、8
                point = (int(x_aver), int(y_aver))
                cv2.circle(frame1, point, point_size, point_color, thickness)

                # 绘制ArUCo检测的边界框
                cv2.line(frame1, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame1, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame1, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame1, bottomLeft, topLeft, (0, 255, 0), 2)
                # 计算并绘制 ArUco 标记的中心 (x, y) 坐标
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame1, (cX, cY), 4, (0, 0, 255), -1)
                # 在框架上绘制 ArUco 标记 ID
                text = '{} dis={:.2f}m num={}'.format(markerID, distance, length)
                cv2.putText(frame1, text, (topLeft[0], topLeft[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                dis_lst[int(markerID) - 4] = distance

            coordinate_4_points = solve(coordinate_w_aruco4, coordinate_w_aruco5, coordinate_w_aruco6,
                                        coordinate_w_aruco7, dis_lst[0], dis_lst[1], dis_lst[2], dis_lst[3])


            # 输出坐标到树莓派
            coordinate_w = coordinate_w_sum / corners_len
            str_out = ';1,{:.2f},{:.2f},{:.2f};'.format(coordinate_w[0], coordinate_w[1], coordinate_w[2])
            cv2.putText(frame1, str_out, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            print('发送前')
            client_socket.sendall(str_out.encode('utf-8'))
            print('成功发送', str_out)

        # --------------------FPS--------------------
        if (time.time() - start_time) != 0:  # 实时显示帧数
            cv2.putText(frame1, "FPS {0}".format(float('%.1f' % (counter / (time.time() - start_time)))), (500, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255),
                        3)
            cv2.imshow("left", frame1)
            counter = 0
            start_time = time.time()
        time.sleep(1 / fps)  # 按原帧率播放

        # cv2.imshow("left", frame1)
        cv2.setMouseCallback(WIN_NAME, onmouse_pick_points, threeD)
        cv2.setMouseCallback('left', onmouse_pick_points, threeD)
        cv2.imshow(WIN_NAME, disp)  # 显示深度图的双目画面

        key = cv2.waitKey(1) & 0xff
        if key == ord("q"):
            break

finally:
    # 关闭连接
    client_socket.close()
    cap.release()
    cv2.destroyAllWindows()
