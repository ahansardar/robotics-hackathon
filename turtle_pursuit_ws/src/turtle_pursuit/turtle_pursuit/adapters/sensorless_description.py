"""Generate safe TurtleBot 4 Lite sensor profiles for a shared Gazebo world."""
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET

RENDER_SENSOR_TYPES = {'camera', 'depth_camera', 'gpu_lidar', 'gpu_ray', 'rgbd_camera'}
SENSOR_PROFILES = {'stable', 'lidar', 'full'}

def strip_render_sensors(xml_text: str) -> str:
    root = ET.fromstring(xml_text)
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'sensor' and child.attrib.get('type') in RENDER_SENSOR_TYPES:
                parent.remove(child); removed += 1
            elif child.tag == 'plugin' and 'sensors-system' in child.attrib.get('filename', ''):
                parent.remove(child); removed += 1
    if removed == 0:
        raise RuntimeError('no render sensors found in installed TurtleBot description')
    return ET.tostring(root, encoding='unicode', xml_declaration=True)

def configure_sensors(xml_text: str, profile: str) -> str:
    """Remove duplicate systems and select stable, lidar, or full sensing.

    Gazebo needs one Sensors system in the world, not one in every robot. This
    Gazebo 8 build cannot instantiate CPU lidar, so we retain one GPU RPLidar per
    robot and remove the redundant cliff / IR GPU rays that caused the original
    render-thread failure. The full profile additionally keeps RGB-D cameras.
    """
    if profile not in SENSOR_PROFILES:
        raise ValueError(f'unknown sensor profile {profile!r}')
    root = ET.fromstring(xml_text)
    found = set()
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'plugin' and 'sensors-system' in child.attrib.get('filename', ''):
                parent.remove(child)
                continue
            if child.tag != 'sensor':
                continue
            sensor_type = child.attrib.get('type', '')
            sensor_name = child.attrib.get('name', '')
            found.add(sensor_name)
            if profile == 'stable' and sensor_type in RENDER_SENSOR_TYPES:
                parent.remove(child)
            elif profile == 'lidar' and sensor_type in {'camera', 'depth_camera', 'rgbd_camera'}:
                parent.remove(child)
            elif sensor_type in {'gpu_lidar', 'gpu_ray'} and sensor_name != 'rplidar':
                parent.remove(child)
            else:
                visualize = child.find('visualize')
                if visualize is not None:
                    visualize.text = 'false'
                if profile == 'full' and sensor_type in {'camera', 'depth_camera', 'rgbd_camera'}:
                    update_rate = child.find('update_rate')
                    if update_rate is None:
                        update_rate = ET.SubElement(child, 'update_rate')
                    update_rate.text = '60'
    if 'rplidar' not in found:
        raise RuntimeError('installed TurtleBot description has no rplidar sensor')
    if profile == 'full' and 'rgbd_camera' not in found:
        raise RuntimeError('installed TurtleBot description has no RGB-D camera')
    return ET.tostring(root, encoding='unicode', xml_declaration=True)

def generate(namespace: str, profile: str = 'stable') -> str:
    xacro = '/opt/ros/jazzy/share/turtlebot4_description/urdf/lite/turtlebot4.urdf.xacro'
    result = subprocess.run(
        ['xacro', xacro, 'gazebo:=ignition', f'namespace:={namespace}'],
        check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    root = ET.fromstring(configure_sensors(result.stdout, profile))
    color = '0.95 0.05 0.05 1' if namespace == 'catcher' else '0.05 0.25 1.0 1'
    marker = ET.SubElement(root, 'link', {'name': 'role_marker'})
    visual = ET.SubElement(marker, 'visual', {'name': 'role_badge'})
    ET.SubElement(visual, 'origin', {'xyz': '0 0 0.55', 'rpy': '0 0 0'})
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'cylinder', {'radius': '0.18', 'length': '0.12'})
    material = ET.SubElement(visual, 'material', {'name': namespace + '_color'})
    ET.SubElement(material, 'color', {'rgba': color})
    if namespace == 'runner':
        zone = ET.SubElement(marker, 'visual', {'name': 'capture_zone'})
        ET.SubElement(zone, 'origin', {'xyz': '0 0 0.035', 'rpy': '0 0 0'})
        zone_geometry = ET.SubElement(zone, 'geometry')
        ET.SubElement(zone_geometry, 'cylinder', {'radius': '0.5', 'length': '0.012'})
        zone_material = ET.SubElement(zone, 'material', {'name': 'capture_zone_blue'})
        ET.SubElement(zone_material, 'color', {'rgba': '0.05 0.35 1.0 0.28'})
    joint = ET.SubElement(root, 'joint', {'name': 'role_marker_joint', 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': 'base_link'})
    ET.SubElement(joint, 'child', {'link': 'role_marker'})
    return ET.tostring(root, encoding='unicode', xml_declaration=True)

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='')
    parser.add_argument('--profile', choices=sorted(SENSOR_PROFILES), default='stable')
    args = parser.parse_args(argv)
    try:
        sys.stdout.write(generate(args.namespace, args.profile))
    except Exception as exc:
        print(f'sensorless description failed: {exc}', file=sys.stderr)
        raise
