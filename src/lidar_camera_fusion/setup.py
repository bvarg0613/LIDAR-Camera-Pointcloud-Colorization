from setuptools import find_packages, setup

package_name = 'lidar_camera_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bvargas',
    maintainer_email='bvargas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colorize_pointcloud = lidar_camera_fusion.colorize_node:main',
            'colorize_pointcloud_fast = lidar_camera_fusion.colorize_node_fast:main',
        ],
    },
)
