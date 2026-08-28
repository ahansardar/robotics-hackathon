import math
import rclpy
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
