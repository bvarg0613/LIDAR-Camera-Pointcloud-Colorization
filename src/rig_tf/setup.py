from setuptools import setup
import os
from glob import glob

package_name = 'rig_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Static TF publisher for LiDAR + camera rig',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_rig_tf = rig_tf.static_rig_tf:main',
        ],
    },
)
