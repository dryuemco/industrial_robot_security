from setuptools import find_packages, setup

package_name = 'enfield_watchdog_static'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yunus Emre Cogurcu',
    maintainer_email='yunusemrecogurcu@gmail.com',
    description='Static Watchdog: IR-level A1-A8 safety rule checker.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'static_watchdog = enfield_watchdog_static.watchdog:main',
    ]},
)
