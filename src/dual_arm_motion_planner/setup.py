from setuptools import find_packages, setup

package_name = 'dual_arm_motion_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/join_target_task.launch.py',  
            'launch/ee_target_task.launch.py',  
            'launch/tcp_target_task.launch.py',   
            'launch/cartesian_collision_task.launch.py',       
        ])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haolei',
    maintainer_email='haoleitong24@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_target_task = dual_arm_motion_planner.dual_arm_motion_planner_q:main',
            'ee_target_task = dual_arm_motion_planner.dual_arm_motion_planner_ee:main',
            'tcp_target_task = dual_arm_motion_planner.dual_arm_motion_planner_tcp:main',
            'collision_target_task = dual_arm_motion_planner.dual_arm_motion_planner_collision:main'
        ],
    },
)
