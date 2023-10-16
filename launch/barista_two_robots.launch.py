import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
import xacro

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    xacro_file = 'barista_robot_model.urdf.xacro'
    package_description = "barista_robot_description"

    ####### DATA INPUT END ##########
    print("Fetching XACRO ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", xacro_file)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    pkg_box_bot_gazebo = get_package_share_directory(package_description)
    install_dir = get_package_prefix(package_description)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'meshes')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"])) 
    

    
    robot_name_1 = "rick"
    robot_name_2 = "morty"

    #Argument launch
    laser_arg = DeclareLaunchArgument(
            name = "include_laser",
            default_value='true',
            description='Include laser in the simulation'
            )

    # robot_name_arg = DeclareLaunchArgument(
    #         name = 'robot_name', 
    #         default_value='robot1', 
    #         description='Name of the robot')


    # # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    
    include_laser_value = LaunchConfiguration('include_laser')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 

    # convert XACRO file into URDF
    doc = xacro.parse(open(robot_desc_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    # Another way to launch xacro file
    urdf_content_1 = Command([
        'xacro', ' ',
        robot_desc_path,' ',
        ' include_laser:=' , include_laser_value, ' ',
        ' robot_name:=', robot_name_1
    ])

    urdf_content_2 = Command([
        'xacro', ' ',
        robot_desc_path,' ',
        ' include_laser:=' , include_laser_value, ' ',
        ' robot_name:=', robot_name_2
    ])

    # Robot State Publisher

    robot_state_publisher_node_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node_1',
        emulate_tty=True,
        namespace=robot_name_1,
        # parameters=[params],
        parameters=[{'frame_prefix': robot_name_1+'/', 'use_sim_time': use_sim_time, 'robot_description': urdf_content_1}],
        output="screen"
    )

    robot_state_publisher_node_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node_2',
        emulate_tty=True,
        namespace=robot_name_2,
        # parameters=[params],
        parameters=[{'frame_prefix': robot_name_2+'/', 'use_sim_time': use_sim_time, 'robot_description': urdf_content_2}],
        output="screen"
    )

    #rviz
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    #spawn robot in Gazebo

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.2]
    position_2 = [1.0, 3.0, 0.2]

    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]

    # Spawn ROBOT Set Gazebo
    spawn_robot_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity_1',
        output='screen',
        arguments=['-entity',
                   'rick',
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', robot_name_1+'/robot_description'
                   ]
    )

    spawn_robot_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity_2',
        output='screen',
        arguments=['-entity',
                   'morty',
                   '-x', str(position_2[0]), '-y', str(position_2[1]
                                                     ), '-z', str(position_2[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', robot_name_2+'/robot_description'
                   ]
    )

    static_tf_pub_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom_1',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', robot_name_1+'/odom']
    )

    static_tf_pub_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom_2',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', robot_name_2+'/odom']
    )

    # create and return launch description object
    return LaunchDescription(
        [                     
            laser_arg,
            # robot_name_arg,
            robot_state_publisher_node_1,
            robot_state_publisher_node_2,
            gazebo,
            spawn_robot_1,
            spawn_robot_2,
            rviz_node,
            static_tf_pub_1,
            static_tf_pub_2
        ]
    )