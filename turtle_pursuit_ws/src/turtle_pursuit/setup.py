from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_pursuit'
setup(
    name=package_name, version='1.0.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools', 'PyYAML'], tests_require=['pytest'], zip_safe=True,
    maintainer='IITBBS Robotics Team', maintainer_email='team@example.com',
    description='Competition-ready TurtleBot pursuit and evasion', license='Apache-2.0',
    entry_points={'console_scripts': [
        'catcher = turtle_pursuit.catcher.node:main',
        'runner = turtle_pursuit.runner.node:main',
        'evaluator = turtle_pursuit.evaluation.node:main',
        'dashboard = turtle_pursuit.evaluation.dashboard:main',
        'benchmark = turtle_pursuit.evaluation.benchmark:main',
        'sensorless_description = turtle_pursuit.adapters.sensorless_description:main',
        'robot_description = turtle_pursuit.adapters.sensorless_description:main',
    ]},
)
