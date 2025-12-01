from setuptools import find_packages, setup
from glob import glob

package_name = 'hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob('hmi/hmidata/data.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madatime',
    maintainer_email='adam.gironcel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hmi_node = hmi.hmi_node:main',
        ],
    },
)
