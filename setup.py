from setuptools import find_packages, setup

package_name = 'mocap_to_px4'

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
    maintainer='Hannibal Paul',
    maintainer_email='ubuntu@todo.todo',
    description='ROS 2 package for MOCAP body to PX4 position information',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'converter = mocap_to_px4.converter_member_function:main',
        ],
    },
)
