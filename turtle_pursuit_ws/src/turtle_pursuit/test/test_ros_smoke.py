import math
import struct
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from turtle_pursuit.catcher.node import CatcherNode
from turtle_pursuit.runner.node import RunnerNode

def test_controller_nodes_start_and_publish_finite_zero_when_stale():
    rclpy.init()
    catcher=CatcherNode(); runner=RunnerNode()
    try:
        catcher.tick(); runner.tick()
        assert math.isfinite(catcher.limiter.last.linear)
        assert math.isfinite(catcher.limiter.last.angular)
        assert catcher.limiter.last.linear == 0.0
        assert runner.limiter.last.linear == 0.0
    finally:
        catcher.destroy_node(); runner.destroy_node(); rclpy.shutdown()

def test_self_pose_falls_back_to_wheel_odometry_without_ground_truth():
    """If sim_ground_truth_pose is never published (e.g. the venue does not expose
    it), each robot must still self-localize from its own wheel odometry instead
    of freezing forever. Odometry starts at (0,0,0) in its own frame; composed
    with the known arena spawn pose it should land back near that spawn point."""
    rclpy.init()
    catcher=CatcherNode(); runner=RunnerNode()
    try:
        odom=Odometry(); odom.pose.pose.orientation.w=1.0
        catcher.adapter._self_odom(odom,'catcher'); runner.adapter._self_odom(odom,'runner')
        cp=catcher.adapter.get_catcher_pose(); rp=runner.adapter.get_runner_pose()
        assert cp is not None and abs(cp.x+1.5)<1e-6 and abs(cp.y)<1e-6
        assert rp is not None and abs(rp.x-1.5)<1e-6 and abs(abs(rp.yaw)-math.pi)<1e-6
    finally:
        catcher.destroy_node(); runner.destroy_node(); rclpy.shutdown()

def test_opponent_pose_falls_back_to_marker_detection_without_ground_truth():
    """With no ground truth at all, the opponent's pose must come from the
    RGB-D camera's colored-marker detection once this robot knows its own pose
    (via the odometry fallback above)."""
    rclpy.init()
    catcher=CatcherNode()
    try:
        odom=Odometry(); odom.pose.pose.orientation.w=1.0
        catcher.adapter._self_odom(odom,'catcher')
        width,height,fx,cx,cy=160,120,120.0,80.0,60.0
        color=Image(); color.height=height; color.width=width; color.encoding='rgb8'; color.step=width*3
        buf=bytearray(color.step*height)
        for y in range(50,70):
            row=y*color.step
            for x in range(70,90):
                o=row+x*3; buf[o],buf[o+1],buf[o+2]=0,0,210
        color.data=bytes(buf)
        depth=Image(); depth.height=height; depth.width=width; depth.encoding='32FC1'; depth.step=width*4
        dbuf=bytearray(depth.step*height)
        for y in range(height):
            row=y*depth.step
            for x in range(width): struct.pack_into('<f',dbuf,row+x*4,2.0)
        depth.data=bytes(dbuf)
        info=CameraInfo(); info.width=width; info.height=height; info.k=[fx,0.,cx,0.,fx,cy,0.,0.,1.]
        catcher.adapter._depth_image(depth); catcher.adapter._camera_information(info)
        catcher.adapter._image(color,'catcher','runner')
        belief=catcher.adapter.get_runner_pose()
        assert belief is not None and abs(belief.x-0.5)<0.05 and abs(belief.y)<0.05
    finally:
        catcher.destroy_node(); rclpy.shutdown()
