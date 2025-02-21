from setuptools import find_packages, setup

package_name = 'robot_planning_pkg'

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
    maintainer='ubuntu',
    maintainer_email='s01087350661@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "astar = robot_planning_pkg.A_star_test:main",
            "testservice = robot_planning_pkg.service_call:main",
            "global_path_planner = robot_planning_pkg.global_path_planner:main",
        ],
    },
)
