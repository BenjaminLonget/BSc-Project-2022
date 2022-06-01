from setuptools import setup

package_name = 'nn_drone_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xilinx',
    maintainer_email='xilinx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'nn_node = nn_drone_final.nn_node:main',
        	'cam_node = nn_drone_final.cam_node:main',
        	'nn_sub_node = nn_drone_final.nn_sub:main',
        	'arm = nn_drone_final.SimpleArmMission:main',
        ],
    },
)
