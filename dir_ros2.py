import os
import shutil
import tkinter as tk
from tkinter import filedialog

# 1. 创建 tkinter 根窗口
root = tk.Tk()
root.withdraw()  # 隐藏主窗口

# 2. 弹出文件夹选择框，选择目标目录
target_directory = filedialog.askdirectory(title="选择目标目录")

# 检查选择的目录是否有效
if not target_directory:
    print("没有选择有效目录。")
else:
    print(f"选择的目录是：{target_directory}")

    # 3. 删除 'launch' 文件夹
    launch_folder = os.path.join(target_directory, 'launch')
    if os.path.exists(launch_folder) and os.path.isdir(launch_folder):
        try:
            shutil.rmtree(launch_folder)
            print("成功删除 'launch' 文件夹。")
        except Exception as e:
            print(f"删除 'launch' 文件夹时出错：{e}")
    else:
        print("'launch' 文件夹不存在。")

    # 4. 删除 'CMakeLists.txt' 文件
    cmake_file = os.path.join(target_directory, 'CMakeLists.txt')
    if os.path.exists(cmake_file) and os.path.isfile(cmake_file):
        try:
            os.remove(cmake_file)
            print("成功删除 'CMakeLists.txt' 文件。")
        except Exception as e:
            print(f"删除 'CMakeLists.txt' 文件时出错：{e}")
    else:
        print("'CMakeLists.txt' 文件不存在。")

    # 5. 删除 'package.xml' 文件
    package_file = os.path.join(target_directory, 'package.xml')
    if os.path.exists(package_file) and os.path.isfile(package_file):
        try:
            os.remove(package_file)
            print("成功删除 'package.xml' 文件。")
        except Exception as e:
            print(f"删除 'package.xml' 文件时出错：{e}")
    else:
        print("'package.xml' 文件不存在。")

    # 6. 创建一个空的 'launch' 文件夹
    try:
        os.makedirs(launch_folder, exist_ok=True)
        print("成功创建空的 'launch' 文件夹。")
    except Exception as e:
        print(f"创建 'launch' 文件夹时出错：{e}")

    # 获取 package_name（目标目录的名称）
    package_name = os.path.basename(target_directory)

    # 7. 创建 display.launch.py 文件的内容
    launch_content = f"""import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('{package_name}')

    # URDF 文件路径
    urdf_file = os.path.join(package_dir, 'urdf', '{package_name}.urdf')

    # 读取 URDF 文件内容
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # 创建 LaunchDescription
    return LaunchDescription([

        # 声明 urdf_file 参数，允许外部传入
        DeclareLaunchArgument('urdf_file', default_value=urdf_file),

        # 启动 joint_state_publisher_gui 节点
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # 启动 robot_state_publisher 节点，并传递 robot_description 参数
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'robot_description': robot_description}}]
        ),

        # 启动 rviz2 节点，并传递 robot_description 参数
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{{'robot_description': robot_description}}]
        ),
    ])
    """

    # 目标文件路径：display.launch.py
    file_path = os.path.join(launch_folder, 'display.launch.py')

    # 将内容写入 display.launch.py 文件
    try:
        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(launch_content)
        print(f"成功将 display.launch.py 文件保存为：{file_path}")
    except Exception as e:
        print(f"写入文件时出错：{e}")

    # 8. 创建 CMakeLists.txt 文件的内容
    cmake_content = f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY launch config meshes urdf
    DESTINATION share/${{PROJECT_NAME}})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
    """

    # 目标文件路径：CMakeLists.txt
    cmake_file_path = os.path.join(target_directory, 'CMakeLists.txt')

    # 将内容写入 CMakeLists.txt 文件
    try:
        with open(cmake_file_path, 'w', encoding='utf-8') as file:
            file.write(cmake_content)
        print(f"成功将 CMakeLists.txt 文件保存为：{cmake_file_path}")
    except Exception as e:
        print(f"写入文件时出错：{e}")

    # 9. 创建 package.xml 文件的内容
    xml_content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="robotsheep@todo.todo">robotsheep</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz2</depend>
  <depend>gazebo_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
    """

    # 目标文件路径：package.xml
    xml_file_path = os.path.join(target_directory, 'package.xml')

    # 将内容写入 package.xml 文件
    try:
        with open(xml_file_path, 'w', encoding='utf-8') as file:
            file.write(xml_content)
        print(f"成功将 package.xml 文件保存为：{xml_file_path}")
    except Exception as e:
        print(f"写入文件时出错：{e}")

    def insert_content_at_line(source_file, target_file, line_number):
        # 读取源文件内容
        with open(source_file, 'r', encoding='utf-8') as src:
            source_content = src.read()

        # 读取目标文件内容
        with open(target_file, 'r', encoding='utf-8') as tgt:
            target_lines = tgt.readlines()  # 读取目标文件的每一行

        # 插入源文件内容到指定行
        # 如果指定的行超出目标文件的行数，内容会被追加到文件末尾
        if line_number > len(target_lines):
            line_number = len(target_lines)

        target_lines.insert(line_number - 1, source_content + '\n')

        # 将修改后的内容写回目标文件
        with open(target_file, 'w', encoding='utf-8') as tgt:
            tgt.writelines(target_lines)

        print(f"内容已成功插入到 {target_file} 的第 {line_number} 行")


    source_file = 'insert_content.txt'
    target_file = target_directory+'/urdf/'+'arm_description.urdf'
    line_number = 7

    insert_content_at_line(source_file, target_file, line_number)
