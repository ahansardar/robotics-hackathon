from ament_index_python.packages import get_package_share_directory
from irobot_create_common_bringup.namespace import GetNamespacedName
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace=LaunchConfiguration('namespace'); world=LaunchConfiguration('world')
    x=LaunchConfiguration('x'); y=LaunchConfiguration('y'); z=LaunchConfiguration('z'); yaw=LaunchConfiguration('yaw')
    robot_name=GetNamespacedName(namespace,'turtlebot4')
    control=get_package_share_directory('irobot_create_control')
    gz_bringup=get_package_share_directory('irobot_create_gz_bringup')
    description=Command(['ros2 run turtle_pursuit sensorless_description --namespace ',namespace])
    control_params=PathJoinSubstitution([control,'config','control.yaml'])
    group=GroupAction([PushRosNamespace(namespace),
        Node(package='robot_state_publisher',executable='robot_state_publisher',name='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':description}],remappings=[('/tf','tf'),('/tf_static','tf_static')]),
        Node(package='ros_gz_sim',executable='create',arguments=['-name',robot_name,'-x',x,'-y',y,'-z',z,'-Y',yaw,'-topic','robot_description'],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([gz_bringup,'/launch/create3_ros_gz_bridge.launch.py']),launch_arguments={'robot_name':robot_name,'dock_name':GetNamespacedName(namespace,'unused_dock'),'namespace':namespace,'world':world}.items()),
        Node(package='controller_manager',executable='spawner',name='joint_state_spawner',arguments=['joint_state_broadcaster','-c','controller_manager','--controller-manager-timeout','30'],output='screen'),
        TimerAction(period=2.0,actions=[Node(package='controller_manager',executable='spawner',name='diffdrive_spawner',arguments=['diffdrive_controller','-c','controller_manager','--controller-manager-timeout','30'],parameters=[control_params],output='screen')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([gz_bringup,'/launch/create3_gz_nodes.launch.py']),launch_arguments={'robot_name':robot_name,'dock_name':GetNamespacedName(namespace,'unused_dock')}.items()),
    ])
    args=[DeclareLaunchArgument('namespace'),DeclareLaunchArgument('world',default_value='pursuit_arena'),DeclareLaunchArgument('x',default_value='0'),DeclareLaunchArgument('y',default_value='0'),DeclareLaunchArgument('z',default_value='0.02'),DeclareLaunchArgument('yaw',default_value='0')]
    return LaunchDescription(args+[group])
