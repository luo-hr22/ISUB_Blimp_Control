import colorsys
import math
import os
import time

import cv2
import numpy as np
import torch
import torch.nn as nn
from PIL import ImageDraw, ImageFont

from nets.yolo import YoloBody
from utils.utils import (cvtColor, get_anchors, get_classes, preprocess_input,
                         resize_image, show_config)
from utils.utils_bbox import DecodeBox
import camera_configs

'''
训练自己的数据集必看注释！
'''

# -----------------------------------双目相机的基本参数---------------------------------------------------------
#   left_camera_matrix          左相机的内参矩阵
#   right_camera_matrix         右相机的内参矩阵
#
#   left_distortion             左相机的畸变系数    格式(K1,K2,P1,P2,0)
#   right_distortion            右相机的畸变系数
# -------------------------------------------------------------------------------------------------------------
# 左镜头的内参，如焦距
left_camera_matrix = np.array([[665.968358, 0.241530, 634.973987], [0, 666.722271, 360.393886], [0, 0, 1]])
right_camera_matrix = np.array([[671.645311, -0.198615, 626.733130], [0, 672.033103, 377.909841], [0, 0, 1]])

# 畸变系数,K1、K2、K3为径向畸变,P1、P2为切向畸变
left_distortion = np.array([[-0.051181929526, -0.0339436888389, 0.00021644011746, 0.0019906243918, 0.0019906243918]])
right_distortion = np.array([[-0.050092214685, -0.041507914265, 0.00009502001542, 0.0011311122168, 0.033998734668]])

# 旋转矩阵
R = np.array([[0.999997697, 0.000809551, 0.001987619],
              [-0.000797875, 0.999982463, -0.005868396],
              [-0.001992335, 0.005866796, 0.999980805]])
# 平移矩阵
T = np.array([-60.94770179, 0.048695591, -0.217858121])

size = (1280, 720)

R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R,
                                                                  T)

# 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
# print(Q)
WIN_NAME = 'Deep disp'
cv2.namedWindow(WIN_NAME, cv2.WINDOW_AUTOSIZE)


class YOLO(object):
    _defaults = {
        # --------------------------------------------------------------------------#
        #   使用自己训练好的模型进行预测一定要修改model_path和classes_path！
        #   model_path指向logs文件夹下的权值文件，classes_path指向model_data下的txt
        #
        #   训练好后logs文件夹下存在多个权值文件，选择验证集损失较低的即可。
        #   验证集损失较低不代表mAP较高，仅代表该权值在验证集上泛化性能较好。
        #   如果出现shape不匹配，同时要注意训练时的model_path和classes_path参数的修改
        # --------------------------------------------------------------------------#
        "model_path": 'logs/last_epoch_weights.pth',
        "classes_path": 'model_data/voc_classes.txt',
        # ---------------------------------------------------------------------#
        #   anchors_path代表先验框对应的txt文件，一般不修改。
        #   anchors_mask用于帮助代码找到对应的先验框，一般不修改。
        # ---------------------------------------------------------------------#
        "anchors_path": 'model_data/yolo_anchors.txt',
        "anchors_mask": [[6, 7, 8], [3, 4, 5], [0, 1, 2]],
        # ---------------------------------------------------------------------#
        #   输入图片的大小，必须为32的倍数。
        # ---------------------------------------------------------------------#
        "input_shape": [640, 640],
        # ------------------------------------------------------#
        #   phi             所使用的YoloV5的版本。n、s、m、l、x
        # ------------------------------------------------------#
        "phi": 's',
        # ---------------------------------------------------------------------#
        #   只有得分大于置信度的预测框会被保留下来
        # ---------------------------------------------------------------------#
        "confidence": 0.5,
        # ---------------------------------------------------------------------#
        #   非极大抑制所用到的nms_iou大小
        # ---------------------------------------------------------------------#
        "nms_iou": 0.3,
        # ---------------------------------------------------------------------#
        #   该变量用于控制是否使用letterbox_image对输入图像进行不失真的resize，
        #   在多次测试后，发现关闭letterbox_image直接resize的效果更好
        # ---------------------------------------------------------------------#
        "letterbox_image": True,
        # -------------------------------#
        #   是否使用Cuda
        #   没有GPU可以设置成False
        # -------------------------------#
        "cuda": True,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    # ---------------------------------------------------#
    #   初始化YOLO
    # ---------------------------------------------------#
    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        for name, value in kwargs.items():
            setattr(self, name, value)
            self._defaults[name] = value

            # ---------------------------------------------------#
        #   获得种类和先验框的数量
        # ---------------------------------------------------#
        self.class_names, self.num_classes = get_classes(self.classes_path)
        self.anchors, self.num_anchors = get_anchors(self.anchors_path)
        self.bbox_util = DecodeBox(self.anchors, self.num_classes, (self.input_shape[0], self.input_shape[1]),
                                   self.anchors_mask)

        # ---------------------------------------------------#
        #   画框设置不同的颜色
        # ---------------------------------------------------#
        hsv_tuples = [(x / self.num_classes, 1., 1.) for x in range(self.num_classes)]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
        self.generate()

        show_config(**self._defaults)

    # ---------------------------------------------------#
    #   生成模型
    # ---------------------------------------------------#
    def generate(self, onnx=False):
        # ---------------------------------------------------#
        #   建立yolo模型，载入yolo模型的权重
        # ---------------------------------------------------#
        self.net = YoloBody(self.anchors_mask, self.num_classes, self.phi)
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net.load_state_dict(torch.load(self.model_path, map_location=device))
        self.net = self.net.eval()
        print('{} model, and classes loaded.'.format(self.model_path))
        if not onnx:
            if self.cuda:
                self.net = nn.DataParallel(self.net)
                self.net = self.net.cuda()

    # ---------------------------------------------------#
    #   检测图片
    # ---------------------------------------------------#
    def detect_image(self, image, client_socket, crop=False, count=False):
        # def detect_image(self, image, crop=False, count=False):

        frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        frame1 = frame[0:720, 0:1280]
        frame2 = frame[0:720, 1280:2560]
        # 将BGR格式转换成灰度图片，用于畸变矫正
        imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # 重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
        # 依据MATLAB测量数据重建无畸变图片,输入图片要求为灰度图
        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)

        # 转换为opencv的BGR格式
        imageL = cv2.cvtColor(img1_rectified, cv2.COLOR_GRAY2BGR)
        imageR = cv2.cvtColor(img2_rectified, cv2.COLOR_GRAY2BGR)

        # BM
        # numberOfDisparities = ((1280 // 8) + 15) & -16  # 640对应是分辨率的宽
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
        # 计算视差
        disparity = stereo.compute(img1_rectified, img2_rectified)

        # 归一化函数算法，生成深度图（灰度图）
        disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # 生成深度图（颜色图）
        dis_color = disparity
        dis_color = cv2.normalize(dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        dis_color = cv2.applyColorMap(dis_color, 2)

        cv2.imshow(WIN_NAME, disp)  # 显示深度图的双目画面

        # 计算三维坐标数据值
        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
        # 计算出的threeD，需要乘以16，才等于现实中的距离
        threeD = threeD * 16

        # ---------------------------------------------------#
        #   计算输入图片的高和宽
        # ---------------------------------------------------#
        image_shape = np.array(np.shape(image)[0:2])
        # box = (0, 0, image_shape[0], image_shape[1]/2)
        box = (0, 0, image_shape[1] / 2, image_shape[0])
        image = image.crop(box)
        image_shape = np.array(np.shape(image)[0:2])
        # ---------------------------------------------------------#
        #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        #   代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
        # ---------------------------------------------------------#
        image = cvtColor(image)
        # ---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        # ---------------------------------------------------------#
        image_data = resize_image(image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        # ---------------------------------------------------------#
        #   添加上batch_size维度
        # ---------------------------------------------------------#
        image_data = np.expand_dims(np.transpose(preprocess_input(np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            # ---------------------------------------------------------#
            #   将图像输入网络当中进行预测！
            # ---------------------------------------------------------#
            outputs = self.net(images)
            outputs = self.bbox_util.decode_box(outputs)
            # ---------------------------------------------------------#
            #   将预测框进行堆叠，然后进行非极大抑制
            # ---------------------------------------------------------#
            results = self.bbox_util.non_max_suppression(torch.cat(outputs, 1), self.num_classes, self.input_shape,
                                                         image_shape, self.letterbox_image, conf_thres=self.confidence,
                                                         nms_thres=self.nms_iou)

            if results[0] is None:
                return image

            top_label = np.array(results[0][:, 6], dtype='int32')
            top_conf = results[0][:, 4] * results[0][:, 5]
            top_boxes = results[0][:, :4]
        # ---------------------------------------------------------#
        #   设置字体与边框厚度
        # ---------------------------------------------------------#
        font = ImageFont.truetype(font='model_data/simhei.ttf',
                                  size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = int(max((image.size[0] + image.size[1]) // np.mean(self.input_shape), 1))
        # ---------------------------------------------------------#
        #   计数
        # ---------------------------------------------------------#
        if count:
            print("top_label:", top_label)
            classes_nums = np.zeros([self.num_classes])
            for i in range(self.num_classes):
                num = np.sum(top_label == i)
                if num > 0:
                    print(self.class_names[i], " : ", num)
                classes_nums[i] = num
            print("classes_nums:", classes_nums)
        # ---------------------------------------------------------#
        #   是否进行目标的裁剪
        # ---------------------------------------------------------#
        if crop:
            for i, c in list(enumerate(top_boxes)):
                top, left, bottom, right = top_boxes[i]
                top = max(0, np.floor(top).astype('int32'))
                left = max(0, np.floor(left).astype('int32'))
                bottom = min(image.size[1], np.floor(bottom).astype('int32'))
                right = min(image.size[0], np.floor(right).astype('int32'))

                dir_save_path = "img_crop"
                if not os.path.exists(dir_save_path):
                    os.makedirs(dir_save_path)
                crop_image = image.crop([left, top, right, bottom])
                crop_image.save(os.path.join(dir_save_path, "crop_" + str(i) + ".png"), quality=95, subsampling=0)
                print("save crop_" + str(i) + ".png to " + dir_save_path)
        # ---------------------------------------------------------#
        #   图像绘制
        # ---------------------------------------------------------#
        for i, c in list(enumerate(top_label)):
            predicted_class = self.class_names[int(c)]
            box = top_boxes[i]
            score = top_conf[i]

            top, left, bottom, right = box

            top = max(0, np.floor(top).astype('int32'))
            left = max(0, np.floor(left).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom).astype('int32'))
            right = min(image.size[0], np.floor(right).astype('int32'))

            # ---------------------------------------------------------#
            x1 = int(np.floor(left))
            x2 = int(np.floor(right))
            y1 = int(np.floor(top))
            y2 = int(np.floor(bottom))
            delta_x = (x2 - x1) / 5
            delta_y = (y2 - y1) / 11
            x_lst = [x1]
            y_lst = [y1]
            distance_lst = []
            for ii in range(0, 5):
                x_lst.append(int(x1 + delta_x * (ii + 1)))
            for ii in range(0, 11):
                y_lst.append(int(y1 + delta_y * (ii + 1)))
                for jj in range(0, 5):
                    distance = math.sqrt(threeD[y_lst[ii]][x_lst[jj]][0] ** 2 +
                                         threeD[y_lst[ii]][x_lst[jj]][1] ** 2 +
                                         threeD[y_lst[ii]][x_lst[jj]][2] ** 2)
                    distance /= 1000
                    if distance < 10:
                        distance_lst.append(distance)
            distance_sum = 0
            for distance in distance_lst:
                distance_sum += distance
            length = len(distance_lst)
            if length == 0:
                distance = 100
            else:
                distance = distance_sum / len(distance_lst)
            # ---------------------------------------------------------#
            '''middle_x = int(np.floor((left + right) / 2))
            middle_y = int(np.floor((top + bottom) / 2))

            print('\n像素坐标 x = %d, y = %d' % (middle_x, middle_y))
            # print("世界坐标是：", threeD[y][x][0], threeD[y][x][1], threeD[y][x][2], "mm")
            print("世界坐标xyz 是：", threeD[middle_y][middle_x][0] / 1000.0, threeD[middle_y][middle_x][1] / 1000.0,
                  threeD[middle_y][middle_x][2] / 1000.0, "m")

            distance = math.sqrt(
                threeD[middle_y][middle_x][0] ** 2 + threeD[middle_y][middle_x][1] ** 2 + threeD[middle_y][middle_x][
                    2] ** 2)
            distance = distance / 1000.0  # mm -> m'''
            # print("距离是：", distance, "m")

            # ---------------------------------------------------------#

            # 从用户获取输入
            client_socket.sendall(('dis=' + str(distance)).encode())

            # 接收响应
            response = client_socket.recv(1024).decode('utf-8')
            print(f"收到响应: {response}")

            label = '{} {:.2f} dis={:.2f}m num={:}'.format(predicted_class, score, distance, length)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label, font)
            label = label.encode('utf-8')
            # print(label, top, left, bottom, right)

            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            for i in range(thickness):
                draw.rectangle([left + i, top + i, right - i, bottom - i], outline=self.colors[c])
            draw.rectangle([tuple(text_origin), tuple(text_origin + label_size)], fill=self.colors[c])
            draw.text(text_origin, str(label, 'UTF-8'), fill=(0, 0, 0), font=font)
            del draw

        return image

    def get_FPS(self, image, test_interval):
        image_shape = np.array(np.shape(image)[0:2])
        # ---------------------------------------------------------#
        #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        #   代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
        # ---------------------------------------------------------#
        image = cvtColor(image)
        # ---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        # ---------------------------------------------------------#
        image_data = resize_image(image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        # ---------------------------------------------------------#
        #   添加上batch_size维度
        # ---------------------------------------------------------#
        image_data = np.expand_dims(np.transpose(preprocess_input(np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            # ---------------------------------------------------------#
            #   将图像输入网络当中进行预测！
            # ---------------------------------------------------------#
            outputs = self.net(images)
            outputs = self.bbox_util.decode_box(outputs)
            # ---------------------------------------------------------#
            #   将预测框进行堆叠，然后进行非极大抑制
            # ---------------------------------------------------------#
            results = self.bbox_util.non_max_suppression(torch.cat(outputs, 1), self.num_classes, self.input_shape,
                                                         image_shape, self.letterbox_image, conf_thres=self.confidence,
                                                         nms_thres=self.nms_iou)

        t1 = time.time()
        for _ in range(test_interval):
            with torch.no_grad():
                # ---------------------------------------------------------#
                #   将图像输入网络当中进行预测！
                # ---------------------------------------------------------#
                outputs = self.net(images)
                outputs = self.bbox_util.decode_box(outputs)
                # ---------------------------------------------------------#
                #   将预测框进行堆叠，然后进行非极大抑制
                # ---------------------------------------------------------#
                results = self.bbox_util.non_max_suppression(torch.cat(outputs, 1), self.num_classes, self.input_shape,
                                                             image_shape, self.letterbox_image,
                                                             conf_thres=self.confidence, nms_thres=self.nms_iou)

        t2 = time.time()
        tact_time = (t2 - t1) / test_interval
        return tact_time

    def detect_heatmap(self, image, heatmap_save_path):
        import cv2
        import matplotlib.pyplot as plt
        def sigmoid(x):
            y = 1.0 / (1.0 + np.exp(-x))
            return y

        # ---------------------------------------------------------#
        #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        #   代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
        # ---------------------------------------------------------#
        image = cvtColor(image)
        # ---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        # ---------------------------------------------------------#
        image_data = resize_image(image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        # ---------------------------------------------------------#
        #   添加上batch_size维度
        # ---------------------------------------------------------#
        image_data = np.expand_dims(np.transpose(preprocess_input(np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            # ---------------------------------------------------------#
            #   将图像输入网络当中进行预测！
            # ---------------------------------------------------------#
            outputs = self.net(images)

        plt.imshow(image, alpha=1)
        plt.axis('off')
        mask = np.zeros((image.size[1], image.size[0]))
        for sub_output in outputs:
            sub_output = sub_output.cpu().numpy()
            b, c, h, w = np.shape(sub_output)
            sub_output = np.transpose(np.reshape(sub_output, [b, 3, -1, h, w]), [0, 3, 4, 1, 2])[0]
            score = np.max(sigmoid(sub_output[..., 4]), -1)
            score = cv2.resize(score, (image.size[0], image.size[1]))
            normed_score = (score * 255).astype('uint8')
            mask = np.maximum(mask, normed_score)

        plt.imshow(mask, alpha=0.5, interpolation='nearest', cmap="jet")

        plt.axis('off')
        plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
        plt.margins(0, 0)
        plt.savefig(heatmap_save_path, dpi=200, bbox_inches='tight', pad_inches=-0.1)
        print("Save to the " + heatmap_save_path)
        plt.show()

    def convert_to_onnx(self, simplify, model_path):
        import onnx
        self.generate(onnx=True)

        im = torch.zeros(1, 3, *self.input_shape).to('cpu')  # image size(1, 3, 512, 512) BCHW
        input_layer_names = ["images"]
        output_layer_names = ["output"]

        # Export the model
        print(f'Starting export with onnx {onnx.__version__}.')
        torch.onnx.export(self.net,
                          im,
                          f=model_path,
                          verbose=False,
                          opset_version=12,
                          training=torch.onnx.TrainingMode.EVAL,
                          do_constant_folding=True,
                          input_names=input_layer_names,
                          output_names=output_layer_names,
                          dynamic_axes=None)

        # Checks
        model_onnx = onnx.load(model_path)  # load onnx model
        onnx.checker.check_model(model_onnx)  # check onnx model

        # Simplify onnx
        if simplify:
            import onnxsim
            print(f'Simplifying with onnx-simplifier {onnxsim.__version__}.')
            model_onnx, check = onnxsim.simplify(
                model_onnx,
                dynamic_input_shape=False,
                input_shapes=None)
            assert check, 'assert check failed'
            onnx.save(model_onnx, model_path)

        print('Onnx model save as {}'.format(model_path))

    def get_map_txt(self, image_id, image, class_names, map_out_path):
        f = open(os.path.join(map_out_path, "detection-results/" + image_id + ".txt"), "w", encoding='utf-8')
        image_shape = np.array(np.shape(image)[0:2])
        # ---------------------------------------------------------#
        #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        #   代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
        # ---------------------------------------------------------#
        image = cvtColor(image)
        # ---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        # ---------------------------------------------------------#
        image_data = resize_image(image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        # ---------------------------------------------------------#
        #   添加上batch_size维度
        # ---------------------------------------------------------#
        image_data = np.expand_dims(np.transpose(preprocess_input(np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            # ---------------------------------------------------------#
            #   将图像输入网络当中进行预测！
            # ---------------------------------------------------------#
            outputs = self.net(images)
            outputs = self.bbox_util.decode_box(outputs)
            # ---------------------------------------------------------#
            #   将预测框进行堆叠，然后进行非极大抑制
            # ---------------------------------------------------------#
            results = self.bbox_util.non_max_suppression(torch.cat(outputs, 1), self.num_classes, self.input_shape,
                                                         image_shape, self.letterbox_image, conf_thres=self.confidence,
                                                         nms_thres=self.nms_iou)

            if results[0] is None:
                return

            top_label = np.array(results[0][:, 6], dtype='int32')
            top_conf = results[0][:, 4] * results[0][:, 5]
            top_boxes = results[0][:, :4]

        for i, c in list(enumerate(top_label)):
            predicted_class = self.class_names[int(c)]
            box = top_boxes[i]
            score = str(top_conf[i])

            top, left, bottom, right = box
            if predicted_class not in class_names:
                continue

            f.write("%s %s %s %s %s %s\n" % (
                predicted_class, score[:6], str(int(left)), str(int(top)), str(int(right)), str(int(bottom))))

        f.close()
        return
