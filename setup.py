from setuptools import find_packages, setup

package_name = 'tracter_odometer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracter_odometer.launch.py', 'launch/tracter_odometer.launch.xml']),
        ('share/' + package_name + '/config', ['config/tracter_config.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/tracter.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nontanan',
    maintainer_email='nontanan@gensurv.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometer_node = tracter_odometer.odometer_node:main',
        ],
    },
)
