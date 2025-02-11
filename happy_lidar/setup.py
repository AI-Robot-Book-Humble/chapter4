from setuptools import setup

package_name = 'happy_lidar'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kosei Demura',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='A happy lidar package',
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'happy_lidar_node = happy_lidar.happy_lidar_node:main',
        ],
    },
)
