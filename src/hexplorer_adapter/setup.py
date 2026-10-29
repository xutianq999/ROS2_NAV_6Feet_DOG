from setuptools import find_packages, setup

package_name = 'hexplorer_adapter'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/adapter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@localhost',
    description='Nav2 栈到 Dobot Hexplorer (miniHex_v2) 的底盘适配层',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hexplorer_adapter_node = '
            'hexplorer_adapter.hexplorer_adapter_node:main',
            'joint_state_bridge = '
            'hexplorer_adapter.joint_state_bridge:main',
        ],
    },
)
