"""Generate the installed TurtleBot 4 Lite URDF without render-backed sensors."""
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET

RENDER_SENSOR_TYPES = {'camera', 'depth_camera', 'gpu_lidar', 'gpu_ray', 'rgbd_camera'}

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

def generate(namespace: str) -> str:
    xacro = '/opt/ros/jazzy/share/turtlebot4_description/urdf/lite/turtlebot4.urdf.xacro'
    result = subprocess.run(
        ['xacro', xacro, 'gazebo:=ignition', f'namespace:={namespace}'],
        check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    root = ET.fromstring(strip_render_sensors(result.stdout))
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
    args = parser.parse_args(argv)
    try:
        sys.stdout.write(generate(args.namespace))
    except Exception as exc:
        print(f'sensorless description failed: {exc}', file=sys.stderr)
        raise
