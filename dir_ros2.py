import os
import shutil
import tkinter as tk
from tkinter import filedialog
import subprocess

PATH_ROOT = os.path.dirname(os.path.abspath(__file__))

# 1. Create tkinter root window
root = tk.Tk()
root.withdraw()  # Hide the main window

# 2. Open folder selection dialog to choose the target directory
target_directory = filedialog.askdirectory(title="Select Target Directory")
# Check if the selected directory is valid
if not target_directory:
    print("No valid directory selected.")
else:
    print(f"Selected directory: {target_directory}")

    path_textures_folder = os.path.join(target_directory, "textures")
    if not os.path.exists(path_textures_folder):
        os.makedirs(path_textures_folder)
    
    path_backup_folder = target_directory + '_backup'
    if os.path.exists(path_backup_folder):
        shutil.rmtree(path_backup_folder)
    shutil.copytree(target_directory, path_backup_folder)

    # 3. Delete the 'launch' folder if it exists
    launch_folder = os.path.join(target_directory, 'launch')
    if os.path.exists(launch_folder) and os.path.isdir(launch_folder):
        try:
            shutil.rmtree(launch_folder)
            print("Successfully deleted the 'launch' folder.")
        except Exception as e:
            print(f"Error deleting 'launch' folder: {e}")
    else:
        print("'launch' folder does not exist.")

    # 4. Delete 'CMakeLists.txt' file if it exists
    cmake_file = os.path.join(target_directory, 'CMakeLists.txt')
    if os.path.exists(cmake_file) and os.path.isfile(cmake_file):
        try:
            os.remove(cmake_file)
            print("Successfully deleted the 'CMakeLists.txt' file.")
        except Exception as e:
            print(f"Error deleting 'CMakeLists.txt' file: {e}")
    else:
        print("'CMakeLists.txt' file does not exist.")

    # 5. Delete 'package.xml' file if it exists
    package_file = os.path.join(target_directory, 'package.xml')
    if os.path.exists(package_file) and os.path.isfile(package_file):
        try:
            os.remove(package_file)
            print("Successfully deleted the 'package.xml' file.")
        except Exception as e:
            print(f"Error deleting 'package.xml' file: {e}")
    else:
        print("'package.xml' file does not exist.")

    # 6. Create an empty 'launch' folder
    try:
        os.makedirs(launch_folder, exist_ok=True)
        print("Successfully created an empty 'launch' folder.")
    except Exception as e:
        print(f"Error creating 'launch' folder: {e}")

    # Get package_name (the name of the target directory)
    package_name = os.path.basename(target_directory)

    # 7. Create display.launch.py file content
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

    # Target file path: display.launch.py
    file_path = os.path.join(launch_folder, 'display.launch.py')

    # Write content to display.launch.py file
    try:
        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(launch_content)
        print(f"Successfully saved 'display.launch.py' as: {file_path}")
    except Exception as e:
        print(f"Error writing to file: {e}")

    # 8. Create CMakeLists.txt file content
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

    # Target file path: CMakeLists.txt
    cmake_file_path = os.path.join(target_directory, 'CMakeLists.txt')

    # Write content to CMakeLists.txt file
    try:
        with open(cmake_file_path, 'w', encoding='utf-8') as file:
            file.write(cmake_content)
        print(f"Successfully saved 'CMakeLists.txt' as: {cmake_file_path}")
    except Exception as e:
        print(f"Error writing to file: {e}")

    # 9. Create package.xml file content
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

    # Target file path: package.xml
    xml_file_path = os.path.join(target_directory, 'package.xml')

    # Write content to package.xml file
    try:
        with open(xml_file_path, 'w', encoding='utf-8') as file:
            file.write(xml_content)
        print(f"Successfully saved 'package.xml' as: {xml_file_path}")
    except Exception as e:
        print(f"Error writing to file: {e}")

    def insert_content_at_line(source_file, target_file, line_number):
        # Read content from source file
        with open(source_file, 'r', encoding='utf-8') as src:
            source_content = src.read()

        # Read target file content
        with open(target_file, 'r', encoding='utf-8') as tgt:
            target_lines = tgt.readlines()  # Read lines from target file

        # Insert source content at the specified line
        # If the line number exceeds the number of lines in the target file, append the content to the end
        if line_number > len(target_lines):
            line_number = len(target_lines)

        target_lines.insert(line_number - 1, source_content + '\n')

        # Write modified content back to target file
        with open(target_file, 'w', encoding='utf-8') as tgt:
            tgt.writelines(target_lines)

        print(
            f"Content successfully inserted into {target_file} at line {line_number}")

    source_file = os.path.join(PATH_ROOT, 'insert_urdf.txt')
    target_file = target_directory + '/urdf/' + \
        os.path.basename(target_directory)+".urdf"
    line_number = 7

    insert_content_at_line(source_file, target_file, line_number)

    # File path
    file_path = target_file

    # Content to replace
    new_first_line = '''<?xml version="1.0" ?>'''

    # Open the file for reading
    with open(file_path, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    if lines:
        lines[0] = new_first_line + '\n'  # Ensure newline is added

    # Write the modified content back to the file
    with open(file_path, 'w', encoding='utf-8') as file:
        file.writelines(lines)

    urdf_folder = os.path.join(target_directory, "urdf")
    command = f"gz sdf -p {target_file} > {urdf_folder}/model.sdf"
    subprocess.run(command, shell=True, check=True)

    source_file = os.path.join(PATH_ROOT, 'insert_sdf.txt')
    target_file = target_directory + '/urdf/' + "model.sdf"
    line_number = 1

    insert_content_at_line(source_file, target_file, line_number)

    # Define file and target directory paths
    source_sdf_file = target_directory + '/urdf/' + "model.sdf"
    target_sdf_directory = os.path.join(os.path.expanduser(
        "~"), ".gazebo", "models", f"{os.path.basename(target_directory)}")

    # Check if target directory exists
    if not os.path.exists(target_sdf_directory):
        # If the directory does not exist, create it
        os.makedirs(target_sdf_directory)
        print(f"Directory {target_sdf_directory} created.")
    else:
        print(f"Directory {target_sdf_directory} already exists.")

    # Perform file copy operation
    target_path = os.path.join(
        target_sdf_directory, os.path.basename(source_sdf_file))
    shutil.copy(source_sdf_file, target_sdf_directory)
    print(f"File {source_sdf_file} successfully copied to {target_sdf_directory}.")

    model_config_content = f"""<?xml version="1.0"?>

<model>
  <name>{os.path.basename(target_directory)}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>todo</name>
    <email>todo@todo.todo/email>
  </author>

  <description>
    sw2urdf ROS2 for gazebo11
  </description>
</model>
    """

    model_config_path = target_sdf_directory
    file_name = "model.config"
    file_path = os.path.join(model_config_path, file_name)
    try:
        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(model_config_content)
        print(f"Successfully saved 'sdf.config' as: {model_config_path}")
    except Exception as e:
        print(f"Error writing to file: {e}")

    source_directory = target_directory+"/meshes"
    destination_directory = target_sdf_directory+"/meshes"

    if os.path.exists(destination_directory):
        print(f"Target directory {destination_directory} already exists!")
    else:
        shutil.copytree(source_directory, destination_directory)
        print(f"Directory {source_directory} successfully copied to {destination_directory}")

    source_directory = target_directory+"/textures"
    destination_directory = target_sdf_directory+"/materials/textures"

    if os.path.exists(destination_directory):
        print(f"Target directory {destination_directory} already exists!")
    else:
        shutil.copytree(source_directory, destination_directory)
        print(f"Directory {source_directory} successfully copied to {destination_directory}")

    file_path = os.path.join(target_directory, "urdf", "model.sdf")

    try:
        os.remove(file_path)  # Delete file
        print(f"File {file_path} successfully deleted.")
    except FileNotFoundError:
        print(f"File {file_path} does not exist.")
    except PermissionError:
        print(f"No permission to delete file {file_path}.")
    except Exception as e:
        print(f"Error deleting file: {e}")

    # Gazebo launch file content
    gazebo_launch_content = f"""import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Get default path
    robot_name_in_model = "{package_name}"
    urdf_tutorial_path = get_package_share_directory('{package_name}')
    default_model_path = os.path.join(
        urdf_tutorial_path, 'urdf', '{package_name}.urdf')

    # Read URDF file content
    with open(default_model_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{{'robot_description': robot_description}}]
    )

    # Include another launch file for Gazebo
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
    )

    # Request Gazebo to spawn the robot
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model])

    return launch.LaunchDescription([
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])
    """

    file_path = os.path.join(launch_folder, 'gazebo.launch.py')

    try:
        with open(file_path, 'w', encoding='utf-8') as file:
            file.write(gazebo_launch_content)
        print(f"Successfully saved 'gazebo.launch.py' as: {file_path}")
    except Exception as e:
        print(f"Error writing to file: {e}")
