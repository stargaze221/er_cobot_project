from setuptools import find_packages, setup

package_name = 'er_cobot'

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
    maintainer='yoonlab01',
    maintainer_email='yoonlab01@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = er_cobot.publisher_member_function:main',
            'listener = er_cobot.subscriber_member_function:main',
            'cobot_server = er_cobot.node_platform_server:main',
            'trajectory_publisher = er_cobot.publisher_trajectory:main',
        ],
    },
)