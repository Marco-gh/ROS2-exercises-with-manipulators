from glob import glob
from setuptools import find_packages, setup

package_name = '6dof_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('6dof_package/launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('6dof_package/urdf/*')),
        ('share/' + package_name + '/rviz', glob('6dof_package/rviz/*')),
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
            '6dof_node = 6dof_package.6dof_node:main'
        ],
    },
)
