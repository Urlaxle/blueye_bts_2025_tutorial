from setuptools import find_packages, setup

package_name = 'bts_docking_guidance'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='urlaxle',
    maintainer_email='waldum94@gmail.com',
    description='Very simple guidance module that just checks if the vehicle is within a sphere of acceptance around predefined waypoints, and if it is publishes next waypoint without any reference model.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bts_docking_guidance = bts_docking_guidance.bts_docking_guidance:main',
        ],
    },
)
