from setuptools import find_packages, setup

package_name = 'speed'

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
    maintainer='bcain',
    maintainer_email='bcain150@gmail.com',
    description='Implements communication with the flipsky ESC over pi5 UART',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "motor_control = speed.motor_control_node:main"
        ],
    },
)
