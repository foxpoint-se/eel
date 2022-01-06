from setuptools import setup

package_name = 'aalen'

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
    maintainer='bulingen',
    maintainer_email='adamlecorney@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss = aalen.gnss.gnss_node:main',
            'imu = aalen.imu.imu_node:main',
            'radio = aalen.radio:main',
        ],
    },
)
