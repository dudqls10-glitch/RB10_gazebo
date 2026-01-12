from setuptools import setup

package_name = 'rbpodo_perception'

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
    maintainer='song',
    maintainer_email='song@todo.todo',
    description='ToF distance processing',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_distance_node = rbpodo_perception.tof_distance_node:main',
            'tof_fov_marker = rbpodo_perception.tof_fov_marker:main',

        ],
    },
)

