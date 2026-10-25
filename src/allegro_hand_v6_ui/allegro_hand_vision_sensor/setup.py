from glob import glob
from setuptools import setup

package_name = 'allegro_hand_vision_sensor'

setup(
    name=package_name, version="1.0.0", packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'README_KR.md']),
        ('share/' + package_name + '/config', [path for path in glob('config/*.yaml') if not path.endswith('/finger_serials.yaml')]),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/docs', ['docs/SDK_REPLACEMENT.md']),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Wonik Robotics', maintainer_email='support@wonikrobotics.com',
    description='SDK-free VISION SENSOR capture and finger mapping.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'vision_sensor_capture = allegro_hand_vision_sensor.capture:main',
        'vision_sensor_mapper = allegro_hand_vision_sensor.mapper:main',
        'vision_sensor_mapper_sweep = allegro_hand_vision_sensor.mapper_sweep:main',
    ]},
)
