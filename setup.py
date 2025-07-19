from setuptools import find_packages, setup

package_name = 'turtlebot3_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', [
            'launch/simple_line_follow.launch.py',
            'launch/stop_sign_line_follow_world.launch.py',
            'launch/two_robots.launch.py'
        ]),

        ('share/' + package_name + '/worlds', [
            'worlds/line_follow_world.world',
            'worlds/wall_world.world'
        ]),

        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uhreed',
    maintainer_email='ureedhussain214@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = turtlebot3_line_follower.line_follower:main',
            'yolo_detect = turtlebot3_line_follower.yolo_detect:main',
            'two_line_follower_node = turtlebot3_line_follower.two_line_follower_node:main',
        ],
    },
)
