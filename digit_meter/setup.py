from setuptools import find_packages, setup

package_name = 'digit_meter'

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
    maintainer='emily',
    maintainer_email='ejhannigan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_viewer = digit_meter.webcam_viewer:main',
            'digit_publisher = digit_meter.digit_publisher:main',
            
        ],
    },
)
