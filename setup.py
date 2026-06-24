from setuptools import find_packages, setup

package_name = 'tiago_visual_servo_docking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ioannis Loizou',
    maintainer_email='yiannisloizou@gmail.com',
    description='Visual servo docking for TIAGo Pro using AprilTag pose feedback.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_servo_control = tiago_visual_servo_docking.visual_servo_control:main',
        ],
    },
)
