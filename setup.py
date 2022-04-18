from setuptools import setup

package_name = 'gps_waypoint_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'utm'],
    zip_safe=True,
    maintainer='Andrew Ealovega',
    maintainer_email='Andrew@Ealovega.dev',
    description='Transformes a file of gps waypoints into waypoints about a map, and then follows those points.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_waypoint_publisher = gps_waypoint_publisher:main'
        ],
    },
)
