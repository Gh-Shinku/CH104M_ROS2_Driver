# CH104M_ROS2_Driver

一个用于 CH104M IMU 传感器的 ROS2 驱动程序，通过串口通信发布 IMU 数据。

## 目录

- [概述](#概述)
- [功能](#功能)
- [硬件要求](#硬件要求)
- [依赖](#依赖)
- [安装](#安装)
- [使用](#使用)

## 概述

此软件包为 CH104M-USB IMU 传感器提供了一个 ROS2 驱动程序。它从串口读取 IMU 数据，并将其作为标准 ROS2 `sensor_msgs/Imu` 消息发布。驱动程序以 921600 波特率进行采集，请在官方（HiPNUC）上位机中对波特率做对应修改。

## 规格

- **标准 ROS2 IMU 消息格式**（`sensor_msgs/msg/Imu`）
- **IMU 数据**：
  - 三轴加速度（g）
  - 三轴角速度（度/秒）
  - 三轴磁场（µT）
  - 欧拉角（RPY，度）
  - 四元数方向（WXYZ）
  - 温度和气压
- **CRC16 数据验证**
- **异步 I/O** 使用 Boost.Asio
- **日志支持** 使用 spdlog

## 依赖
```bash
# Boost
sudo apt install libboost-all-dev

# spdlog（日志库）
sudo apt install libspdlog-dev

# fmt（格式化库）
sudo apt install libfmt-dev
```

## 安装

1. **创建 ROS2 工作空间**（如果尚未创建）：
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **克隆仓库**：
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Gh-Shinku/CH104M_ROS2_Driver.git
   ```

3. **构建软件包**：
   ```bash
   colcon build --packages-select imu_publisher
   ```

4. **加载工作空间**：
   ```bash
   source install/setup.bash
   ```

## 使用

### 基本使用

1. **连接 CH104M IMU** 到 USB 端口（通常显示为 `/dev/CH104M-USB`）

2. **检查设备权限**：
   ```bash
   # 将用户添加到 dialout 组
   sudo usermod -a -G dialout $USER
   # 注销并重新登录以使更改生效
   
   # 笔者使用 udev 将串口设备重命名成 /dev/CH104M-USB，之后会支持指定串口名
   ```
   如果需要修改串口设备，暂且编辑 `src/imu_publisher.cpp` 中的源代码：

    ```cpp
    serial_port_(ioc_, "/dev/ttyUSB0", std::bind(&ImuPublisher::publish_imu_data, this, std::placeholders::_1)),
    ```

3. **运行 IMU 发布程序**：
   ```bash
   ros2 run imu_publisher imu_publisher
   ```

4. **验证数据发布**：
   ```bash
   ros2 topic echo /imu
   ```
