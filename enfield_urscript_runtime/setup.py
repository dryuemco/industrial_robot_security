from setuptools import find_packages, setup

package_name = 'enfield_urscript_runtime'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/t001_smoke.launch.py']),
        ('share/' + package_name + '/scripts',
            ['scripts/run_t001_smoke.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yunus Emre Cogurcu',
    maintainer_email='yunusemrecogurcu@gmail.com',
    description=(
        'URScript runtime nodes: publish translator output to ur_robot_driver '
        'and record URSim telemetry. Simulation-only.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'urscript_publisher = '
        'enfield_urscript_runtime.urscript_publisher_node:main',
        'telemetry_recorder = '
        'enfield_urscript_runtime.telemetry_recorder_node:main',
    ]},
)
