# SW2URDF ROS2

## 环境配置

1. windows系统
2. python
3. tk

## 操作说明

1. 目录版

运行dir_ros2.py选择sw2urdf生成的目录，自动生成。

2. zip版

把sw2urdf生成的目录压缩为zip，运行zip_ros2.py选择zip文件，自动生成。（推荐，防止原文件被破环）

## 运行测试

把生成的目录放到ROS2工作空间的src目录下，colcon build然后source工作空间。

运行 **display.launch.py** launch文件



> 注：arm_description目录为测试目标



