from setuptools import find_packages, setup

package_name = 'fishbot_application'

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
    maintainer='elge',
    maintainer_email='1197860601@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "init_robot_pose = fishbot_application.init_robot_pose:main",
            "get_robot_pose = fishbot_application.get_robot_pose:main",
            "nav_to_pose = fishbot_application.nav_to_pose:main",
            "waypoint_follow = fishbot_application.waypoint_follow:main",
        ],
    },
)
