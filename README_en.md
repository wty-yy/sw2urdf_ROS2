[中文](README.md) | [English](README_en.md)

Original repository link [GitHub - fish1sheep/sw2urdf_ROS2](https://github.com/fish1sheep/sw2urdf_ROS2). For detailed usage, please refer to its README.

## Fork Modifications
1. First, save the original ROS1 `path/to/urdf_proj` folder as `path/to/urdf_proj_backup` because the code will directly modify the original folder.
2. Automatically create an empty `textures` folder (if it doesn't exist) to avoid errors.

## Usage Instructions
1. Use the [ros/solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter) plugin to export the ROS1 URDF project folder, denoted as `urdf_proj`.
    ```bash
    # The format exported by solidworks_urdf_exporter should be as follows
    path/to/urdf_proj
    ├── CMakeLists.txt
    ├── config
    ├── export.log
    ├── launch
    ├── meshes
    ├── package.xml
    ├── textures  # It's okay if this doesn't exist
    └── urdf
    ```
2. Modify the `base_link` in `insert_urdf.py` to the name of your robot's base link (the first link set, default is `base_link`). This is necessary because `base_footprint` needs to be created as the base, and the `xyz` offset required for `base_footprint_joint` should be set according to the robot's needs.
3. In the ROS2 workspace, execute `python3 dir_ros2.py`, **navigate to** the `urdf_proj` folder, click confirm, and if the following information is returned, it indicates success:
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
4. Copy the converted ROS2 URDF project folder `urdf_proj` to the ROS2 workspace path (e.g., `/ros2_ws/src`), and execute:
```bash
cp -r path/to/urdf_proj /ros2_ws/src
cd /ros2_ws
colcon build --symlink-install
source ./install/setup.sh

ros2 launch urdf_proj display.launch.py  # Start RVIZ2 to display the robot
# or
ros2 launch urdf_proj gazebo.launch.py  # Start GAZEBO simulation
```