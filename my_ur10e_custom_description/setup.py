from setuptools import find_packages, setup

package_name = 'my_ur10e_custom_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_ur10e.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/ur10e_with_prismatic.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/initial_positions.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
