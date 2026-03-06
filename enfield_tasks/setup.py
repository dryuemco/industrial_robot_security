from setuptools import find_packages, setup
from glob import glob

package_name = 'enfield_tasks'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ir/schema', glob('ir/schema/*.json')),
        ('share/' + package_name + '/ir/tasks', glob('ir/tasks/*.json')),
    ],
    install_requires=['setuptools', 'jsonschema', 'pyyaml'],
    zip_safe=True,
    maintainer='Yunus Emre Cogurcu',
    maintainer_email='yunusemrecogurcu@gmail.com',
    description='Task IR definitions, JSON schema, and validators.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
