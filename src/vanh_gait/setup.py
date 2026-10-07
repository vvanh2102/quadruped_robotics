from setuptools import find_packages, setup
from glob import glob

package_name = 'vanh_gait'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='v005101',
    maintainer_email='vvanh2102@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_gait_node = vanh_gait.gait_node:main',
        ],
    },
)
