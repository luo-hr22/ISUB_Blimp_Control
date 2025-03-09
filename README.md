ISUB_Blimp_Control
# Flight control algorithms for Project ISUB (Intelligent Small Unmanned Blimp)

# ISUB_Blimp_Control 🚁

**Intelligent Small Unmanned Blimp (ISUB)** 是一个基于智慧视觉的小型飞艇平台，专为复杂空间环境下的动态监测设计。项目融合双目视觉、深度学习与自主控制技术，提供高效、低成本的工业巡检与家庭安全监测解决方案。

---

## ✨ 主要特性
- **轻量化设计**：流线型气囊（CFD优化）与矢量推进系统，载重120g，续航>30分钟。
- **自主定位**：双目视觉 + ArUco标记实现3米内定位误差≤5%。
- **智能识别**：YOLOv5算法训练危险点（如插座）识别，置信度>0.9。
- **双模控制**：蓝牙手动控制 + PID自动控制，支持实时模式切换。
- **低延迟通信**：基于Socket协议，数据传输延迟≤1秒。

---

## 🛠️ 硬件设计
- **核心元件**：
  - **树莓派 Zero 2 W**：运行控制程序与数据传输。
  - **双目摄像头**：实时视频流与深度计算。
  - **矢量推进系统**：双电机+舵机，180°转向。
  - **JY901B十轴传感器**：高精度姿态角测量。
- **结构**：3D打印ABS框架，碳纤维连杆，氦气囊抗风设计。

![Design](media/image2.png)  
*图：飞艇硬件设计图（[图片2.png](#)）*

---

## 🖥️ 软件架构
```plaintext
ISUB_Blimp_Control/
├── aruco/                 # ArUco标记识别与定位
│   ├── camera_configs.py  # 双目相机标定参数
│   └── stereo_detection.py
├── raspberry_pi/          # 树莓派控制代码
│   ├── i2cnew/            # I2C通信与传感器驱动
│   │   ├── PID.c          # PID控制算法
│   │   └── wit_c_sdk.c    # 十轴传感器SDK
├── yolo/                  # 目标检测模型
│   ├── train.py           # YOLOv5训练脚本
│   └── predict.py         # 实时推理脚本
└── docs/                  # 项目文档与仿真结果
