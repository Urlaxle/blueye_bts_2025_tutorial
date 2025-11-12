from setuptools import find_packages, setup

package_name = 'bts_blueye_handler'

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
    description='Connects to the Blueye vehicle through the Blueye python SDK or GZ and subscribes to a topic listening for WrenchStamped messages to control the vehicle motion.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bts_blueye_handler = bts_blueye_handler.bts_blueye_handler:main',
            "bts_blueye_simulator_handler = bts_blueye_handler.bts_blueye_simulator_handler:main",
        ],
    },
)
