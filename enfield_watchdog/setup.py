from setuptools import find_packages, setup
from glob import glob

package_name = 'enfield_watchdog'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Yunus Emre Cogurcu',
    maintainer_email='yunusemrecogurcu@gmail.com',
    description='Runtime safety monitors for ENFIELD.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'velocity_monitor = enfield_watchdog.velocity_monitor_node:main',
            'zone_monitor = enfield_watchdog.zone_monitor_node:main',
            'safety_aggregator = enfield_watchdog.safety_aggregator_node:main',
        ],
    },
)
