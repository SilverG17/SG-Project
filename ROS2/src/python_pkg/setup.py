from setuptools import find_packages, setup
from glob import glob
package_name = 'python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf') + glob('urdf/*.xacro') + glob('urdf/*.sdf')),
        ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/*.dae')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhhy2507',
    maintainer_email='huy250704@gmail.com',
    description='Simulated line world with white ground and 26mm black line for Ignition Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
