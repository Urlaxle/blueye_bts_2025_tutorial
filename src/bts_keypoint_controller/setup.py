from setuptools import find_packages, setup

package_name = 'bts_keypoint_controller'

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
    description='Keyboard controller publishing keystrokes as a WrenchStamped message to control Blueye',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bts_keyboard_controller = bts_keypoint_controller.bts_keypoint_controller:main'
        ],
    },
)
