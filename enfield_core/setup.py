from setuptools import find_packages, setup

package_name = 'enfield_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'lark-parser'],
    zip_safe=True,
    maintainer='Yunus Emre Cogurcu',
    maintainer_email='yunusemrecogurcu@gmail.com',
    description='ENFIELD core: RAPID parser, IR validators, shared utilities.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'watchdog_node = enfield_core.watchdog_node:main',
        ],
    },
)
