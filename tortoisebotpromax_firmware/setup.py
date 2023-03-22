from setuptools import setup

package_name = 'tortoisebotpromax_firmware'

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
    maintainer='kartik',
    maintainer_email='kartiksoni010@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_publisher = tortoisebotpromax_firmware.differential_publisher:main',
            'ticks_to_odom = tortoisebotpromax_firmware.ticks_to_odom:main',
            'differential_array_pub = tortoisebotpromax_firmware.differential_array_pub:main',
        ],
    },
)
