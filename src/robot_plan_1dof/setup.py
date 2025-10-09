from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_plan_1dof'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('robot_plan_1dof/launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('robot_plan_1dof/urdf/*')),
        ('share/' + package_name + '/rviz', glob('robot_plan_1dof/rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='marcosilveri32@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={  
        'console_scripts': [
            'talker = robot_plan_1dof.publisher_function:main',
            'listener = robot_plan_1dof.subscriber_function:main',
        ],
    },
)
