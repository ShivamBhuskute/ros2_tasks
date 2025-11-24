from setuptools import find_packages, setup

package_name = 'robocon_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shivam',
    maintainer_email='bhuskuteshivam8@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "camera_publisher = robocon_vision_pkg.camera_publisher:main",
            "zone_detector = robocon_vision_pkg.zone_detector:main",
        ],
    },
)
