# AiNex

[English](https://github.com/Hiwonder/ainex/blob/main/README.md) | 中文

<p align="center">
  <img src="./sources/images/ainex.png" alt="AiNex Logo" width="600"/>
</p>

## 产品概述

AiNex是幻尔科技开发的先进四足机器人平台，基于ROS（机器人操作系统）构建。具备精密的运动学控制、传感器集成和AI功能，AiNex为机器人研究、教育和开发提供了全面的平台。

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
