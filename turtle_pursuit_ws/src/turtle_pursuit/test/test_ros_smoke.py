import math
import struct
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from turtle_pursuit.catcher.node import CatcherNode
from turtle_pursuit.runner.node import RunnerNode
from turtle_pursuit.evaluation.node import EvaluatorNode
from turtle_pursuit.common.geometry import Pose2D

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

def test_fallback_active_flag_resets_so_a_second_dropout_logs_again():
    """The fallback-active flag gates the diagnostic warning, not the pose
    fallback itself (that's gated purely by ground-truth freshness) -- but if
    it never resets when ground truth recovers, a flicker (drop, recover,
    drop again) mid-match would only ever log its first occurrence, hiding a
    second one from whoever is watching the console at the venue."""
    rclpy.init()
    catcher=CatcherNode()
    try:
        odom=Odometry(); odom.pose.pose.orientation.w=1.0
        catcher.adapter._self_odom(odom,'catcher')
        assert catcher.adapter._odom_fallback_active['catcher'] is True
        fresh=Odometry(); fresh.pose.pose.position.x=-1.5; fresh.pose.pose.orientation.w=1.0
        catcher.adapter._odom(fresh,catcher.adapter.catcher,'catcher')
        assert catcher.adapter._odom_fallback_active['catcher'] is False
    finally:
        catcher.destroy_node(); rclpy.shutdown()

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

def test_catcher_keeps_pursuing_a_stale_target_instead_of_freezing():
    """Losing the opponent's pose beyond stale_timeout (camera lost the marker,
    no ground truth) previously froze the robot completely for the rest of the
    match -- 'SEARCH' mode did not actually search, it just stopped, and could
    never recover since a stationary robot's fixed-FOV camera can't reacquire
    a target it isn't currently facing. It must keep pursuing the last known
    position instead (flagged via a STALE_TARGET mode suffix), since a moving
    robot has some chance of reacquiring or closing distance and a frozen one
    has none. Only truly missing data (own pose, or an opponent estimate that
    has literally never arrived) should still produce zero motion."""
    import time
    rclpy.init()
    node=CatcherNode()
    try:
        now=node.adapter.now()
        node.adapter.catcher.pose=Pose2D(-1.5,0.,0.,now); node.adapter.catcher.received=now
        node.adapter.runner.pose=Pose2D(1.5,0.,math.pi,now-100); node.adapter.runner.received=now-100
        modes=[]; node.mode_pub.publish=lambda m:modes.append(m.data)
        for _ in range(20):
            node.tick(); time.sleep(.05)
        assert 'STALE_TARGET' in modes[-1] and modes[-1]!='WAITING'
        assert node.limiter.last.linear>0.1  # genuinely moving, not frozen
    finally:
        node.destroy_node(); rclpy.shutdown()

def test_evaluator_invalidates_capture_hold_across_a_stale_observability_gap():
    """RULEBOOK.md requires an UNBROKEN 1.0s hold inside the 0.5m capture
    radius. If the evaluator's own pose tracking goes stale for a moment (a
    ground-truth/sensor hiccup), it must not silently trust that the hold
    stayed continuous through the gap it couldn't observe -- the Runner could
    have escaped past the radius and come back entirely inside the blind
    spot. Confirmed this was previously possible: a partial pre-gap hold plus
    an unobserved escape-and-return produced a false CAPTURED result the
    instant tracking resumed, purely from elapsed wall-clock time."""
    rclpy.init()
    node=EvaluatorNode()
    try:
        fake_now=[0.0]; fake_stale=[False]
        node.adapter.now=lambda:fake_now[0]
        node.adapter.stale=lambda timeout:fake_stale[0]
        node.adapter.catcher.pose=Pose2D(0.,0.,0.,0.)
        node.adapter.runner.pose=Pose2D(0.3,0.,0.,0.)  # within the 0.5m radius

        fake_now[0]=0.3; node.tick()
        assert node.detector.entered==0.3 and not node.done

        fake_stale[0]=True
        node.adapter.runner.pose=Pose2D(2.0,0.,0.,0.)  # escapes, unobserved
        fake_now[0]=1.0; node.tick()
        assert node.detector.entered is None  # the gap must invalidate the accruing hold

        node.adapter.runner.pose=Pose2D(0.3,0.,0.,0.)  # back within radius, still stale
        fake_now[0]=2.0; node.tick()
        fake_stale[0]=False
        fake_now[0]=2.0; node.tick()
        assert not node.done  # must NOT award a capture based on the unobserved gap

        # A genuinely continuous, fully-observed hold afterward must still capture.
        fake_now[0]=2.05; node.tick()
        fake_now[0]=2.5; node.tick()
        fake_now[0]=3.05; node.tick()
        assert node.done
    finally:
        node.destroy_node(); rclpy.shutdown()
