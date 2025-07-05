from setuptools import find_packages, setup

package_name = 'depth_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/depth_map.launch.py',
                                               'launch/point.launch.py',
                                               'launch/octomap.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'matplotlib',  # For real-time visualization
        'numpy',       # Numeric operations
        'opencv-python'  # CvBridge Python binding
    ],
    zip_safe=True,
    maintainer='caddie',
    maintainer_email='caddie@todo.todo',
    description='Depth image to 2D occupancy grid generator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_map_node = depth_map.depth_map_node:main',
            'point_map_node = depth_map.point_map_node:main',
            'filter_node = depth_map.filter:main'
        ],
    },
)
