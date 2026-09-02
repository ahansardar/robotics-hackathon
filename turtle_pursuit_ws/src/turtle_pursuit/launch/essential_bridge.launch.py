from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace=LaunchConfiguration('namespace'); robot=LaunchConfiguration('robot_name'); dock=LaunchConfiguration('dock_name'); world=LaunchConfiguration('world'); sensors=LaunchConfiguration('sensors')
    toolbox=get_package_share_directory('irobot_create_gz_bringup')
    pose_params=PathJoinSubstitution([toolbox,'config','pose_republisher_params.yaml'])
    sensor_params=PathJoinSubstitution([toolbox,'config','sensors_params.yaml'])
    gz_model=['/model/',robot]
    gz_lidar=['/world/',world,'/model/',robot,'/link/rplidar_link/sensor/rplidar/scan']
    gz_camera=['/world/',world,'/model/',robot,'/link/oakd_rgb_camera_frame/sensor/rgbd_camera']
    nodes=[
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='cmd_vel_bridge',output='screen',arguments=[
            [namespace,'/cmd_vel@geometry_msgs/msg/TwistStamped[ignition.msgs.Twist'],
            [*gz_model,'/cmd_vel@geometry_msgs/msg/TwistStamped]ignition.msgs.Twist']],remappings=[([namespace,'/cmd_vel'],'cmd_vel'),([*gz_model,'/cmd_vel'],'diffdrive_controller/cmd_vel')]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='pose_bridge',output='screen',arguments=[[*gz_model,'/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']],remappings=[([*gz_model,'/pose'],'_internal/sim_ground_truth_pose')]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='odom_base_tf_bridge',output='screen',arguments=[[*gz_model,'/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']],remappings=[([*gz_model,'/tf'],'tf')]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='bumper_contact_bridge',output='screen',arguments=[[namespace,'/bumper_contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts']],remappings=[([namespace,'/bumper_contact'],'bumper_contact')]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='lidar_bridge',output='screen',arguments=[[*gz_lidar,'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']],remappings=[(gz_lidar,'scan')],condition=IfCondition(NotEqualsSubstitution(sensors,'stable'))),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='camera_bridge',output='screen',arguments=[
            [*gz_camera,'/image@sensor_msgs/msg/Image[gz.msgs.Image'],
            [*gz_camera,'/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
            [*gz_camera,'/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']],remappings=[
            ([*gz_camera,'/image'],'camera/color/image_raw'),
            ([*gz_camera,'/depth_image'],'camera/depth/image_raw'),
            ([*gz_camera,'/camera_info'],'camera/color/camera_info')],condition=IfCondition(EqualsSubstitution(sensors,'full'))),
        Node(package='irobot_create_gz_toolbox',executable='pose_republisher_node',name='pose_republisher_node',parameters=[pose_params,{'robot_name':robot,'dock_name':dock,'use_sim_time':True}],output='screen'),
        Node(package='irobot_create_gz_toolbox',executable='sensors_node',name='sensors_node',parameters=[sensor_params,{'use_sim_time':True}],output='screen'),
    ]
    args=[DeclareLaunchArgument('namespace'),DeclareLaunchArgument('robot_name'),DeclareLaunchArgument('dock_name',default_value='unused_dock'),DeclareLaunchArgument('world',default_value='pursuit_arena'),DeclareLaunchArgument('sensors',default_value='stable',choices=['stable','lidar','full'])]
    return LaunchDescription(args+nodes)
