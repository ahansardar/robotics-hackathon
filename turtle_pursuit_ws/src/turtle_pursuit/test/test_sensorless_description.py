from turtle_pursuit.adapters.sensorless_description import strip_render_sensors

def test_strips_render_sensors_and_sensor_system_only():
    xml='''<robot><gazebo><plugin filename="libgz-sim-sensors-system.so"/><plugin filename="keep.so"/></gazebo><gazebo><sensor type="gpu_lidar"/><sensor type="imu"/></gazebo></robot>'''
    output=strip_render_sensors(xml)
    assert 'gpu_lidar' not in output
    assert 'sensors-system' not in output
    assert 'type="imu"' in output
    assert 'keep.so' in output
