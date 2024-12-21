import zipfile
import os
import shutil
import tkinter as tk
from tkinter import filedialog


# Unzip ZIP file, clean up old files, and create necessary files
def unzip_and_cleanup(zip_path, target_dir):
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(target_dir)
        print(f"ZIP file extracted to: {target_dir}")

    # Delete unnecessary files and folders
    delete_files_and_folders(target_dir)

    # Create 'launch' folder
    create_launch_folder(target_dir)

    # Create launch file
    create_launch_file(target_dir + os.path.splitext(zip_file_name)
                       [0] + "/launch/", os.path.splitext(zip_file_name)[0])

    # Create CMakeLists.txt file
    create_cmake_file(target_dir + os.path.splitext(zip_file_name)
                      [0], os.path.splitext(zip_file_name)[0])

    # Create package.xml file
    create_xml_file(target_dir + os.path.splitext(zip_file_name)
                    [0], os.path.splitext(zip_file_name)[0])

    source_file = 'insert_content.txt'
    target_file = target_dir + \
        os.path.splitext(zip_file_name)[0] + '/urdf/' + 'arm_description.urdf'
    line_number = 7

    insert_content_at_line(source_file, target_file, line_number)

# Delete unnecessary files and folders


def delete_files_and_folders(target_dir):
    launch_dir = os.path.join(
        target_dir + os.path.splitext(zip_file_name)[0], 'launch')
    if os.path.exists(launch_dir):
        shutil.rmtree(launch_dir)
        print(f"Deleted folder: {launch_dir}")

    files_to_delete = ['CMakeLists.txt', 'package.xml']
    for file in files_to_delete:
        file_path = os.path.join(
            target_dir + os.path.splitext(zip_file_name)[0], file)
        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"Deleted file: {file_path}")

# Create 'launch' folder


def create_launch_folder(target_dir):
    launch_dir = os.path.join(
        target_dir + os.path.splitext(zip_file_name)[0], 'launch')
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)
        print(f"Created 'launch' folder: {launch_dir}")
    else:
        print(f"'launch' folder already exists: {launch_dir}")

# Choose ZIP file


def choose_zip_file():
    root = tk.Tk()
    root.withdraw()  # Hide the root window
    zip_file_path = filedialog.askopenfilename(
        title="Select ZIP file",
        filetypes=[("ZIP Files", "*.zip")]
    )
    return zip_file_path

# Create display.launch.py file


def create_launch_file(launch_dir, package_name):
    launch_content = f"""import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('{package_name}')

    # URDF file path
    urdf_file = os.path.join(package_dir, 'urdf', '{package_name}.urdf')

    # Read URDF file content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Create LaunchDescription
    return LaunchDescription([

        # Declare urdf_file argument to allow external input
        DeclareLaunchArgument('urdf_file', default_value=urdf_file),

        # Launch joint_state_publisher_gui node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Launch robot_state_publisher node with robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'robot_description': robot_description}}]
        ),

        # Launch rviz2 node with robot_description parameter
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
    print(f"Created display.launch.py file: {launch_file_path}")

# Create CMakeLists.txt file


def create_cmake_file(cmake_dir, package_name):
    cmake_content = f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY launch config meshes urdf
	DESTINATION share/${{PROJECT_NAME}})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
"""
    cmake_file_path = os.path.join(cmake_dir, 'CMakeLists.txt')
    with open(cmake_file_path, 'w', encoding='utf-8') as file:
        file.write(cmake_content)
    print(f"Created CMakeLists.txt file: {cmake_file_path}")

# Create package.xml file


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
    print(f"Created package.xml file: {xml_file_path}")

# Insert content into a specific line of the target file


def insert_content_at_line(source_file, target_file, line_number):
    # Read source file content
    with open(source_file, 'r', encoding='utf-8') as src:
        source_content = src.read()

    # Read target file content
    with open(target_file, 'r', encoding='utf-8') as tgt:
        target_lines = tgt.readlines()  # Read each line from target file

    # Insert source content at the specified line
    # If the line number exceeds the target file's line count, append content at the end
    if line_number > len(target_lines):
        line_number = len(target_lines)

    target_lines.insert(line_number - 1, source_content + '\n')

    # Write the modified content back to the target file
    with open(target_file, 'w', encoding='utf-8') as tgt:
        tgt.writelines(target_lines)

    print(
        f"Content successfully inserted into {target_file} at line {line_number}")


# Main program
zip_path = choose_zip_file()
if zip_path:
    zip_file_name = os.path.basename(zip_path)
    target_dir = os.path.join('./')
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    unzip_and_cleanup(zip_path, target_dir)
else:
    print("No ZIP file selected")
