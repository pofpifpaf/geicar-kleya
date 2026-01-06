from setuptools import find_packages, setup

package_name = 'adas'

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
            'collision_avoidance_node = adas.collision_avoidance_node:main',
            'airbag_shock_detection = adas.airbag_shock_detect:main',
            'adas_priority_node = adas.adas_priority_node:main',
            'esp_node = adas.esp_node:main',
            'lane_centering_assist_node = adas.lane_centering_assist:main',
        ],
    },
)
