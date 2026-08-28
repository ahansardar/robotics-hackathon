import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    tb=get_package_share_directory('turtlebot4_gz_bringup'); desc=get_package_share_directory('turtlebot4_description'); irobot=get_package_share_directory('irobot_create_description'); gz=get_package_share_directory('ros_gz_sim'); pkg=get_package_share_directory('turtle_pursuit')
    world=LaunchConfiguration('world'); headless=LaunchConfiguration('headless'); rviz=LaunchConfiguration('rviz'); seed=LaunchConfiguration('seed'); duration=LaunchConfiguration('match_duration'); cstrategy=LaunchConfiguration('catcher_strategy'); rstrategy=LaunchConfiguration('runner_strategy'); result=LaunchConfiguration('result_file')
    args=[DeclareLaunchArgument('headless',default_value='true'),DeclareLaunchArgument('rviz',default_value='false'),DeclareLaunchArgument('world',default_value='pursuit_arena'),DeclareLaunchArgument('seed',default_value='1'),DeclareLaunchArgument('match_duration',default_value='180.0'),DeclareLaunchArgument('catcher_strategy',default_value='predictive'),DeclareLaunchArgument('runner_strategy',default_value='strategic'),DeclareLaunchArgument('result_file',default_value='/tmp/turtle_pursuit_result.json'),DeclareLaunchArgument('startup_delay',default_value='25.0')]
    resource=SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',':'.join([os.path.join(pkg,'worlds'),os.path.join(tb,'worlds'),str(Path(desc).parent),str(Path(irobot).parent)]))
    gz_head=IncludeLaunchDescription(PythonLaunchDescriptionSource([gz,'/launch/gz_sim.launch.py']),launch_arguments={'gz_args':[world,'.sdf -r -s -v 2']}.items(),condition=IfCondition(headless))
    gz_gui=IncludeLaunchDescription(PythonLaunchDescriptionSource([gz,'/launch/gz_sim.launch.py']),launch_arguments={'gz_args':[world,'.sdf -r -v 2 --gui-config ',PathJoinSubstitution([pkg,'config','gazebo_gui.config'])]}.items(),condition=UnlessCondition(headless))
    spawn=PathJoinSubstitution([pkg,'launch','sensorless_spawn.launch.py'])
    def robot(ns,x,y,yaw): return IncludeLaunchDescription(PythonLaunchDescriptionSource([spawn]),launch_arguments={'namespace':ns,'x':x,'y':y,'z':'0.02','yaw':yaw,'world':world}.items())
    clock=Node(package='ros_gz_bridge',executable='parameter_bridge',name='clock_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    nodes=[Node(package='turtle_pursuit',executable='catcher',parameters=[os.path.join(pkg,'config','pursuit.yaml'),{'strategy':cstrategy}]),Node(package='turtle_pursuit',executable='runner',parameters=[os.path.join(pkg,'config','pursuit.yaml'),{'strategy':rstrategy,'seed':ParameterValue(seed,value_type=int)}]),Node(package='turtle_pursuit',executable='evaluator',parameters=[os.path.join(pkg,'config','pursuit.yaml'),{'match_duration':ParameterValue(duration,value_type=float),'result_file':result}]),Node(package='rviz2',executable='rviz2',condition=IfCondition(rviz))]
    delayed=TimerAction(period=LaunchConfiguration('startup_delay'),actions=nodes)
    runner_spawn=TimerAction(period=15.0,actions=[robot('runner','1.5','0.0','3.14159')])
    return LaunchDescription(args+[resource,gz_head,gz_gui,robot('catcher','-1.5','0.0','0.0'),runner_spawn,clock,delayed])
