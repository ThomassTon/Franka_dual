from setuptools import find_packages, setup

package_name = 'franka_dual_arm_sine_wave'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sin_wave_task.launch.py',            
        ]),
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
            'sine_wave_joint_publisher = franka_dual_arm_sine_wave.sine_wave_joint_publisher:main'
        ],
    },
)
