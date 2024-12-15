import zipfile
import os
import shutil
import tkinter as tk
from tkinter import filedialog


# 解压 ZIP 文件并清理旧文件，创建相关文件
def unzip_and_cleanup(zip_path, target_dir):
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(target_dir)
        print(f"ZIP 文件已解压到：{target_dir}")

    # 删除不需要的文件和文件夹
    delete_files_and_folders(target_dir)

    # 创建 launch 文件夹
    create_launch_folder(target_dir)

    # 创建 launch 文件
    create_launch_file(target_dir + os.path.splitext(zip_file_name)
                       [0] + "/launch/", os.path.splitext(zip_file_name)[0])

    # 创建 CMakeLists.txt 文件
    create_cmake_file(target_dir + os.path.splitext(zip_file_name)
                      [0], os.path.splitext(zip_file_name)[0])

    # 创建 package.xml 文件
    create_xml_file(target_dir + os.path.splitext(zip_file_name)
                    [0], os.path.splitext(zip_file_name)[0])

    source_file = 'insert_content.txt'
    target_file = target_dir + \
        os.path.splitext(zip_file_name)[0]+'/urdf/'+'arm_description.urdf'
    line_number = 7

    insert_content_at_line(source_file, target_file, line_number)

# 删除不需要的文件和文件夹


def delete_files_and_folders(target_dir):
    launch_dir = os.path.join(
        target_dir + os.path.splitext(zip_file_name)[0], 'launch')
    if os.path.exists(launch_dir):
        shutil.rmtree(launch_dir)
        print(f"已删除文件夹：{launch_dir}")

    files_to_delete = ['CMakeLists.txt', 'package.xml']
    for file in files_to_delete:
        file_path = os.path.join(
            target_dir + os.path.splitext(zip_file_name)[0], file)
        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"已删除文件：{file_path}")


# 创建 launch 文件夹
def create_launch_folder(target_dir):
    launch_dir = os.path.join(
        target_dir + os.path.splitext(zip_file_name)[0], 'launch')
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)
        print(f"已创建 'launch' 文件夹：{launch_dir}")
    else:
        print(f"'launch' 文件夹已存在：{launch_dir}")


# 选择 ZIP 文件
def choose_zip_file():
    root = tk.Tk()
    root.withdraw()  # 隐藏根窗口
    zip_file_path = filedialog.askopenfilename(
        title="选择 ZIP 文件",
        filetypes=[("ZIP Files", "*.zip")]
    )
    return zip_file_path


# 创建 display.launch.py 文件
def create_launch_file(launch_dir, package_name):
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
    launch_file_path = os.path.join(launch_dir, 'display.launch.py')
    with open(launch_file_path, 'w', encoding='utf-8') as file:
        file.write(launch_content)
    print(f"已创建 display.launch.py文件：{launch_file_path}")


# 创建 CMakeLists.txt 文件
def create_cmake_file(cmake_dir, package_name):
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
    cmake_file_path = os.path.join(cmake_dir, 'CMakeLists.txt')
    with open(cmake_file_path, 'w', encoding='utf-8') as file:
        file.write(cmake_content)
    print(f"已创建 CMakeLists.txt文件：{cmake_file_path}")


# 创建 package.xml 文件
def create_xml_file(xml_dir, package_name):
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
    xml_file_path = os.path.join(xml_dir, 'package.xml')
    with open(xml_file_path, 'w', encoding='utf-8') as file:
        file.write(xml_content)
    print(f"已创建 package.xml文件：{xml_file_path}")


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


# 主程序
zip_path = choose_zip_file()
if zip_path:
    zip_file_name = os.path.basename(zip_path)
    target_dir = os.path.join('./')
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    unzip_and_cleanup(zip_path, target_dir)
else:
    print("未选择 ZIP 文件")
