from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler,IncludeLaunchDescription,TimerAction,ExecuteProcess
from launch.event_handlers import OnProcessStart,OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import Command,PathJoinSubstitution,LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    
    pkg_arm ='robot_desc'
    urdf_file='robot_model.urdf.xacro'

    pkg_description=FindPackageShare(pkg_arm) 

    use_sim_time = LaunchConfiguration('use_sim_time')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_camera = LaunchConfiguration('use_camera')
    use_rviz = LaunchConfiguration('use_rviz')

     #Declaration of Gazebo and world

    #Declare the world file
    world_file_name = 'empty_world.sdf'
    world = os.path.join(get_package_share_directory('robot_gazebo'), 'world', world_file_name)
    
    
    #Launch configuration variables
    gz_args = LaunchConfiguration('gz_args')


    #Declare Gazebo launch arguments
    declare_gz_args_cmd = DeclareLaunchArgument(
          name = 'gz_args',
          default_value = ['-r -v 4 ',world,' --physics-engine gz-physics-bullet-featherstone-plugin'],
          description = 'Defining the world for robot model'
     )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Uses simulated clock when set to true'
    )

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_camera_cmd = DeclareLaunchArgument(
        name='use_camera',
        default_value='false',
        description='Flag to enable the RGBD camera for Gazebo point cloud simulation')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Flag to enable RViz')

    # Initialization of directories
    xacro_file_GZ =PathJoinSubstitution([pkg_description,'urdf','robots',urdf_file]) 
          
    robot_description_content=ParameterValue(Command([
          'xacro',' ',xacro_file_GZ,' ',
          'use_sim_time:=',use_sim_time,' ',
          'jsp_gui:=',jsp_gui,' ',
          'use_camera:=',use_camera,' ',
          'use_rviz:=',use_rviz
        ]),
          value_type=str)
     
    robot_controllers=PathJoinSubstitution([FindPackageShare('robot_gazebo'),'config','simple_controller.yaml'])
    
    #default_ros_gz_bridge_config_file_path=PathJoinSubstitution([FindPackageShare('robot_gazebo'),'config','ros_gz_bridge_config.yaml'])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        #parameters=[{'config_file': default_ros_gz_bridge_config_file_path}],
        output='screen'
    )

    # Includes optimizations to minimize latency and bandwidth when streaming image data 
    #gz_bridge_camera = Node(
    #    package='ros_gz_image',
    #   executable='image_bridge',
    #    arguments=[
    #       '/camera_head/depth_image',
    #        '/camera_head/image',
    #    ],
    #    remappings=[
    #        ('/camera_head/depth_image', '/camera_head/depth/image_rect_raw'),
    #       ('/camera_head/image', '/camera_head/color/image_raw'),
    #    ],
    #)

    #Combined all together creating the gazebo node
    gazebo_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'])
               ]),
                launch_arguments={'gz_args': gz_args}.items()
          )
    # Robot State Publisher to generate the /robot state topic with URDF data
    robot_state_publisher=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_painter_arm',
		output='screen',
		parameters=[{
               'use_sim_time' : use_sim_time,
			'robot_description':robot_description_content,
            'jsp_gui': jsp_gui,
            #'use_camera': use_camera,
            'use_rviz': use_rviz,
            }]
     )

     #Spawn the robot
    spawn_entity_robot=Node(
        package='ros_gz_sim',
        executable='create',
        arguments   = ['-topic', 'robot_description' , '-name', 'panda'],
        output='screen'
     )

    joint_state_broadcaster_spawner=Node(
          package="controller_manager",
          executable="spawner",
          arguments=['joint_state_broadcaster']
     )
    
    controller_spawner=Node(
          package="controller_manager",
          executable="spawner",
          arguments=["arm_controller",
          '--param-file',robot_controllers
          ]
     )   
     # Start gripper action controller
    start_gripper_action_controller_cmd = Node(
        package="controller_manager",   
        executable="spawner",
        arguments=["gripper_action_controller",
        '--param-file',robot_controllers,
        ],
        output='screen')
   
    return LaunchDescription([   
          declare_gz_args_cmd,
          declare_use_sim_time_cmd,
          declare_jsp_gui_cmd,
          declare_use_camera_cmd,
          declare_use_rviz_cmd,
          bridge,
          gazebo_launch,
          spawn_entity_robot,
          robot_state_publisher,
          joint_state_broadcaster_spawner,
          controller_spawner,
          start_gripper_action_controller_cmd,
    ])