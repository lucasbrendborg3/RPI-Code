from setuptools import find_packages, setup

package_name = 'load_cell'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
        'rclpy',
        'std_msgs',
        'hx711-gpiozero',
        'pyserial'],
    zip_safe=True,
    maintainer='arm',
    maintainer_email='lucas@brendborg.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_cell_node = load_cell.load_cell_node:main',
            'jacobian_logger = load_cell.jacobian_logger:main',
        ],
    },
)
