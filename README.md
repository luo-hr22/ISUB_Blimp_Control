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

## 🚀 快速开始
1. 依赖安装：
pip install opencv-python flask numpy torch
2. 硬件连接：树莓派接入电机、舵机、传感器与摄像头。
3. 启动程序：
cd raspberry_pi/i2cnew && make && ./main
4. 控制指令：
* W/A/S/D：前进/左转/后退/右转
* J/U：舵机正转/反转
* 空格：回正悬停

## 🎯 项目成果
* 专利：在申2项（危险点排查飞艇、三维建模平台）。
* 性能：最大速度2m/s，30秒稳定悬停，实时避障。
* 应用场景：工业巡检、家庭安防、大型活动监测。

Blimp
图：飞艇实物图（图片1.png）

## 🌟 未来计划
* 集成激光雷达避障
* SSH远程操控优化
* 移动目标跟随（如行人追踪）

## 🛎️ 贡献与许可
贡献：欢迎提交Issue或PR！详细指南见 CONTRIBUTING.md。

许可：Apache 2.0 License © 2025 ISUB Team.

```text
  ___====-_  _-====___
 _| ~~~--_ \/ _--~~~ _|
|          \/          |
|  -= ISUB =-\/        |
 \_..____./  \__..___/
   // || \\   // || \\
  ~~~~~~     ~~~~~~~

