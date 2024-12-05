from setuptools import setup,find_packages

package_name = 'kdl_ik_service'

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonas',
    maintainer_email='jdech@uni-bremen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_ros_server = scripts.start_ros_server:main',
        ],
    },
)