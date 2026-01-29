# AiNex

[English](https://github.com/Hiwonder/ainex/blob/main/README.md) | 中文

<p align="center">
  <img src="./sources/images/ainex.png" alt="AiNex Logo" width="600"/>
</p>

## 产品概述

### 关于AiNex

AiNex的核心是一个简单而深刻的愿景：将研究实验室中复杂的人形机器人转变为每个开发者、学生和研究人员都能使用的"伙伴"。它不仅仅是一个硬件套件，更是一个基于机器人操作系统（ROS）构建的完全开源的智能实体。我们相信，真正的创新发生在算法走出仿真，学会在真实物理世界中"站立"和"行走"的时候。AiNex正是为此而生——一个24自由度的双足机器人，旨在成为您探索移动机器人、计算机视觉和人工智能最激动人心的实验平台。

### 为类人智能设计的高性能硬件

AiNex设计的核心是赋予它类人的移动能力。以Raspberry Pi 4B/5作为计算大脑，由24个智能串行总线舵机驱动，实现精确灵活的姿态控制。双足行走、跨越障碍物和复杂手势都在其能力范围内。集成的2自由度高清摄像头和可开合的灵巧机械手构成了它感知和与世界互动的基础。所有这些都安装在坚固的金属框架内，确保动态任务期间的稳定性。

### 由开源算法驱动的核心能力

AiNex的智能源于其开放的软件栈。其灵魂是先进的逆运动学算法，实现全向运动、灵活的步态规划和实时姿态调整，完成行走、转向和爬楼梯等流畅自然的动作。结合基于OpenCV的机器视觉，AiNex完成精确的颜色/物体识别、视觉跟踪、巡线和智能分拣等高级AI应用。从相机捕获到认知决策再到协调全身运动，形成完整的智能闭环。

### 你的项目，无限可能

AiNex是一个真正的开发平台。我们提供完整的ROS源代码、丰富的教程和充满活力的社区，支持您从基础运动控制到前沿AI研究的旅程。无论您是探索双足运动的强化学习、开发人机交互系统，还是原型服务机器人应用，AiNex都准备好将您的愿景变为现实。

## 官方资源

### 幻尔科技官方

- **官方网站**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **产品页面**: [https://www.hiwonder.com/products/ainex](https://www.hiwonder.com/products/ainex)
- **官方文档**: [https://docs.hiwonder.com/projects/AiNex/en/latest/](https://docs.hiwonder.com/projects/AiNex/en/latest/)
- **技术支持**: support@hiwonder.com

## 核心功能

### ROS集成

- **基于ROS架构** - 构建在机器人操作系统上，具有模块化和灵活性
- **多个启动文件** - 便捷的启动和配置管理
- **服务集成** - Systemd服务实现自动启动
- **Rosbridge支持** - WebSocket接口用于远程控制

### 高级控制

- **运动学控制** - 精确的逆运动学实现流畅运动
- **舵机控制器** - 高精度舵机电机控制系统
- **行走参数** - 可配置的步态参数适应不同地形
- **姿态管理** - 初始姿态配置和管理

### 传感器校准

- **IMU校准** - 惯性测量单元校准工具
- **磁力计校准** - 磁传感器校准支持
- **相机校准** - 视觉系统校准工具

### 应用支持

- **手机APP集成** - 通过智能手机应用程序进行远程控制
- **OLED显示** - OLED屏幕实时状态显示
- **自定义脚本** - 可扩展的基于Python的脚本系统

## 仓库结构

```
ainex/
└── src/
    ├── ainex_app/              # 应用层
    │   ├── launch/             # 启动文件
    │   └── scripts/            # 应用脚本
    ├── ainex_bringup/          # 系统启动
    │   ├── launch/             # 启动文件
    │   ├── scripts/            # 启动脚本
    │   └── service/            # Systemd服务
    ├── ainex_calibration/      # 校准工具
    │   ├── config/             # 校准配置
    │   └── launch/             # 校准启动文件
    └── ainex_driver/           # 硬件驱动
        └── ainex_kinematics/   # 运动学控制
            ├── config/         # 控制器配置
            └── launch/         # 驱动启动文件
```

## 快速开始

### 前置要求

- ROS Noetic（或兼容版本）
- Python 3.x
- 所需的ROS包

### 安装

1. 克隆仓库：
```bash
git clone https://github.com/Hiwonder/ainex.git
cd ainex
```

2. 构建工作空间：
```bash
catkin_make
source devel/setup.bash
```

3. 启动系统：
```bash
roslaunch ainex_bringup bringup.launch
```

## 使用方法

### 基础控制

启动基础控制器：
```bash
roslaunch ainex_bringup base.launch
```

### 校准

运行IMU校准：
```bash
roslaunch ainex_calibration imu_calibration.launch
```

运行相机校准：
```bash
roslaunch ainex_calibration camera_calibration.launch
```

### 应用模式

启动应用节点：
```bash
roslaunch ainex_app start.launch
```

## 社区与支持

- **GitHub Issues**: 报告问题和请求功能
- **邮件支持**: support@hiwonder.com
- **文档资料**: 全面的指南和教程

## 许可证

本项目开源，可用于教育和研究目的。

---

**幻尔科技** - 赋能机器人教育创新
