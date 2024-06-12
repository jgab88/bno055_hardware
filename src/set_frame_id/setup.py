from setuptools import setup

package_name = 'set_frame_id'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Set frame_id for IMU messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_frame_id_node = set_frame_id.set_frame_id_node:main'
        ],
    },
)
