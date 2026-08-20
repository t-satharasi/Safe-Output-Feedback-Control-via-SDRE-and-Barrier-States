from setuptools import find_packages, setup
from glob import glob

package_name = 'lyla_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['lyla_controller/config_LyLA.json']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
        ('share/' + package_name + '/rviz',
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saia',
    maintainer_email='saia@todo.todo',
    description='LyLA Controller for PX4 quadrotor',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'lyla_node = lyla_controller.LyLA_node:main',
            'lyla_viz = lyla_controller.lyla_viz:main',
        ],
    },
)
