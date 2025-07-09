from setuptools import find_packages, setup

package_name = 'ros_qr_reader'

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
    maintainer='voltanie',
    maintainer_email='voltanie@todo.todo',
    description='TODO: Package description',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub = ros_qr_reader.qr_sub:main',
            'qr_code_service = ros_qr_reader.qr_srv:main',
            'qr_code_client = ros_qr_reader.qr_cli:main'
        ],
    },
)
