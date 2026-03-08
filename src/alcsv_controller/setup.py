from setuptools import setup

package_name = 'alcsv_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gtpc1',
    maintainer_email='gtpc1@todo.todo',
    description='Controller nodes for ALCSV robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heading_controller_node = alcsv_controller.heading_controller_node:main',
            'robot_sim_node = alcsv_controller.robot_sim_node:main',
            'simulated_robot_node = alcsv_controller.simulated_robot_node:main',
            'waypoint_follower_node = alcsv_controller.waypoint_follower_node:main',
        ],
    },
)
