from setuptools import find_packages, setup
from glob import glob

package_name = 'prova_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(f'{package_name}/launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob(f'{package_name}/urdf/*')),
        ('share/' + package_name + '/world', glob(f'{package_name}/world/test.sdf')),
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
        ],
    },
)
