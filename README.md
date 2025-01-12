原仓库连接[GitHub - fish1sheep/sw2urdf_ROS2](https://github.com/fish1sheep/sw2urdf_ROS2)详细使用方法可以参考其README

## Fork修改内容
1. 将原始ROS1的`path/to/urdf_proj`文件夹先另存为`path/to/urdf_proj_backup`，因为代码会直接修改原始文件夹
2. 自动创建`textures`空文件夹（如果没有），避免报错

## 使用方法
1. 使用[ros/solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter)插件导出为ROS1的URDF项目文件夹，记为`urdf_proj`
    ```bash
    # 用solidworks_urdf_exporter到处的格式应该如下
    path/to/urdf_proj
    ├── CMakeLists.txt
    ├── config
    ├── export.log
    ├── launch
    ├── meshes
    ├── package.xml
    ├── textures  # 可能没有也没关系
    └── urdf
    ```
2. 修改`insert_urdf.py`中的`base_link`为你机器人基准link的名称（第一个设置的link，默认叫`base_link`，因为需要创建base_footprint作为基准，`base_footprint_joint`所需的`xyz`偏移量根据机器人需要设定）
3. 在ROS2的工作空间下，执行`python3 dir_ros2.py`，**进入到**`urdf_proj`文件夹，点击确认，返回如下信息则说明成功：
    ```bash
    Selected directory: path/to/urdf_proj
    Successfully deleted the 'launch' folder.
    Successfully deleted the 'CMakeLists.txt' file.
    Successfully deleted the 'package.xml' file.
    Successfully created an empty 'launch' folder.
    Successfully saved 'display.launch.py' as: path/to/urdf_proj/launch/display.launch.py
    Successfully saved 'CMakeLists.txt' as: path/to/urdf_proj/CMakeLists.txt
    Successfully saved 'package.xml' as: path/to/urdf_proj/package.xml
    Content successfully inserted into path/to/urdf_proj/urdf/urdf_proj.urdf at line 7
    Content successfully inserted into path/to/urdf_proj/urdf/model.sdf at line 1
    Directory /root/.gazebo/models/urdf_proj already exists.
    File path/to/urdf_proj/urdf/model.sdf successfully copied to /root/.gazebo/models/urdf_proj.
    Successfully saved 'sdf.config' as: /root/.gazebo/models/urdf_proj
    Target directory /root/.gazebo/models/urdf_proj/meshes already exists!
    Target directory /root/.gazebo/models/urdf_proj/materials/textures already exists!
    File path/to/urdf_proj/urdf/model.sdf successfully deleted.
    Successfully saved 'gazebo.launch.py' as: path/to/urdf_proj/launch/gazebo.launch.py
    ```
4. 将转换完毕的ROS2的URDF项目文件夹`urdf_proj`复制到ROS2的工作路径下（例如`/ros2_ws/src`），执行
```bash
cd /ros2_ws
colcon build --symlink-install
source ./install/setup.sh

ros2 launch urdf_proj display.launch.py  # 启动RVIZ2显示机器人
# 或者
ros2 launch urdf_proj gazebo.launch.py  # 启动GAZEBO仿真
```