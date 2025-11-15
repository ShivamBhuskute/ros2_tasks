from setuptools import find_packages, setup

package_name = 'node_comm_first'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_pipeline.launch.py']),
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
            "sensor_node = node_comm_first.sensor_node:main",
            "processor_node = node_comm_first.processor_node:main",
            "logger_node = node_comm_first.logger_node:main"
        ],
    },
)
