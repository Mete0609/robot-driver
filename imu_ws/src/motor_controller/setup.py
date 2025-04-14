from setuptools import find_packages, setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/yj_bot.urdf']),
        ('share/' + package_name + '/config', ['config/diff_drive_controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mete',
    maintainer_email='mete@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = motor_controller.motor_controller_node:main',
            'controller_bridge_node = motor_controller.controller_bridge_node:main',
        ],
    },
)
