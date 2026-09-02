from turtle_pursuit.adapters.sensorless_description import configure_sensors, strip_render_sensors

def test_strips_render_sensors_and_sensor_system_only():
    xml='''<robot><gazebo><plugin filename="libgz-sim-sensors-system.so"/><plugin filename="keep.so"/></gazebo><gazebo><sensor type="gpu_lidar"/><sensor type="imu"/></gazebo></robot>'''
    output=strip_render_sensors(xml)
    assert 'gpu_lidar' not in output
    assert 'sensors-system' not in output
    assert 'type="imu"' in output
    assert 'keep.so' in output

def test_full_profile_keeps_camera_and_only_primary_gpu_lidar():
    xml='''<robot><gazebo><plugin filename="libgz-sim-sensors-system.so"/><sensor name="cliff_front" type="gpu_lidar"/><sensor name="rplidar" type="gpu_lidar"><visualize>true</visualize></sensor><sensor name="rgbd_camera" type="rgbd_camera"><update_rate>30</update_rate><visualize>true</visualize></sensor></gazebo></robot>'''
    output=configure_sensors(xml,'full')
    assert 'sensors-system' not in output
    assert 'name="rplidar" type="gpu_lidar"' in output
    assert 'cliff_front' not in output
    assert 'name="rgbd_camera" type="rgbd_camera"' in output
    assert '<update_rate>60</update_rate>' in output
    assert '<visualize>false</visualize>' in output

def test_lidar_profile_removes_camera():
    xml='''<robot><gazebo><sensor name="rplidar" type="gpu_lidar"/><sensor name="rgbd_camera" type="rgbd_camera"/></gazebo></robot>'''
    output=configure_sensors(xml,'lidar')
    assert 'rplidar' in output and 'rgbd_camera' not in output
