import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    tb=get_package_share_directory('turtlebot4_gz_bringup'); desc=get_package_share_directory('turtlebot4_description'); irobot=get_package_share_directory('irobot_create_description'); gz=get_package_share_directory('ros_gz_sim'); pkg=get_package_share_directory('turtle_pursuit')
    world=LaunchConfiguration('world'); world_name=LaunchConfiguration('world_name'); headless=LaunchConfiguration('headless'); rviz=LaunchConfiguration('rviz'); dashboard=LaunchConfiguration('dashboard'); scenario=LaunchConfiguration('scenario'); sensors=LaunchConfiguration('sensors'); seed=LaunchConfiguration('seed'); duration=LaunchConfiguration('match_duration'); cstrategy=LaunchConfiguration('catcher_strategy'); rstrategy=LaunchConfiguration('runner_strategy'); result=LaunchConfiguration('result_file'); catcher_speed=LaunchConfiguration('catcher_max_linear'); runner_speed=LaunchConfiguration('runner_max_linear')
    args=[DeclareLaunchArgument('headless',default_value='true'),DeclareLaunchArgument('rviz',default_value='false'),DeclareLaunchArgument('dashboard',default_value='false'),DeclareLaunchArgument('scenario',default_value='Competition Match'),DeclareLaunchArgument('sensors',default_value='full',choices=['stable','lidar','full']),DeclareLaunchArgument('world',default_value='pursuit_arena'),DeclareLaunchArgument('world_name',default_value='pursuit_arena'),DeclareLaunchArgument('seed',default_value='1'),DeclareLaunchArgument('match_duration',default_value='180.0'),DeclareLaunchArgument('catcher_strategy',default_value='predictive'),DeclareLaunchArgument('runner_strategy',default_value='strategic'),DeclareLaunchArgument('catcher_max_linear',default_value='0.70'),DeclareLaunchArgument('runner_max_linear',default_value='0.70'),DeclareLaunchArgument('result_file',default_value='/tmp/turtle_pursuit_result.json'),DeclareLaunchArgument('startup_delay',default_value='25.0')]
    resource=SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',':'.join([os.path.join(pkg,'worlds'),os.path.join(tb,'worlds'),str(Path(desc).parent),str(Path(irobot).parent)]))
    gz_head=IncludeLaunchDescription(PythonLaunchDescriptionSource([gz,'/launch/gz_sim.launch.py']),launch_arguments={'gz_args':[world,'.sdf -r -s -v 2']}.items(),condition=IfCondition(headless))
    gz_gui=IncludeLaunchDescription(PythonLaunchDescriptionSource([gz,'/launch/gz_sim.launch.py']),launch_arguments={'gz_args':[world,'.sdf -r -v 2 --gui-config ',PathJoinSubstitution([pkg,'config','gazebo_gui.config'])]}.items(),condition=UnlessCondition(headless))
    spawn=PathJoinSubstitution([pkg,'launch','sensorless_spawn.launch.py'])
    def robot(ns,x,y,yaw): return IncludeLaunchDescription(PythonLaunchDescriptionSource([spawn]),launch_arguments={'namespace':ns,'x':x,'y':y,'z':'0.02','yaw':yaw,'world':world_name,'sensors':sensors}.items())
    clock=Node(package='ros_gz_bridge',executable='parameter_bridge',name='clock_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    sensor_required=ParameterValue(NotEqualsSubstitution(sensors,'stable'),value_type=bool)
    camera_required=ParameterValue(EqualsSubstitution(sensors,'full'),value_type=bool)
    nodes=[Node(package='turtle_pursuit',executable='catcher',parameters=[os.path.join(pkg,'config','pursuit.yaml'),{'strategy':cstrategy,'max_linear':ParameterValue(catcher_speed,value_type=float),'require_lidar':sensor_required,'require_camera':camera_required}]),Node(package='turtle_pursuit',executable='runner',parameters=[os.path.join(pkg,'config','pursuit.yaml'),{'strategy':rstrategy,'seed':ParameterValue(seed,value_type=int),'max_linear':ParameterValue(runner_speed,value_type=float),'require_lidar':sensor_required,'require_camera':camera_required}]),Node(package='turtle_pursuit',executable='evaluator',parameters=[os.path.join(pkg,'config','pursuit.yaml'),{'match_duration':ParameterValue(duration,value_type=float),'result_file':result}]),Node(package='rviz2',executable='rviz2',condition=IfCondition(rviz))]
    delayed=TimerAction(period=LaunchConfiguration('startup_delay'),actions=nodes)
    dashboard_node=Node(package='turtle_pursuit',executable='dashboard',parameters=[{'scenario':scenario}],condition=IfCondition(dashboard),output='screen')
    runner_spawn=TimerAction(period=15.0,actions=[robot('runner','1.5','0.0','3.14159')])
    return LaunchDescription(args+[resource,gz_head,gz_gui,robot('catcher','-1.5','0.0','0.0'),runner_spawn,clock,dashboard_node,delayed])
