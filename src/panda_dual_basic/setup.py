from setuptools import setup

package_name = 'panda_dual_basic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/dual_panda_gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='your@email.com',
    description='Dual Panda Gazebo Launch Demo',
    license='MIT',
    tests_require=['pytest'],
)