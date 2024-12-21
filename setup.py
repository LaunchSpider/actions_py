from setuptools import find_packages, setup

package_name = 'actions_py'

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
    maintainer='launchspider',
    maintainer_email='danylo.bezruchenko@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "count_until_server = actions_py.count_until_server:main",
            "count_until_client = actions_py.count_until_client:main",
            "position_velocity_action_server = actions_py.position_velocity_action_server:main",
            "position_velocity_action_client = actions_py.position_velocity_action_client:main",
            "position_velocity_action_client_2 = actions_py.position_velocity_action_client_2:main"
        ],
    },
)
