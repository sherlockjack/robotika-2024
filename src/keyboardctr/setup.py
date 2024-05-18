from setuptools import find_packages, setup

package_name = 'keyboardctr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'rclpy',
    'pynput'
],
    zip_safe=True,
    maintainer='sherlockjack',
    maintainer_email='sherlocknagata@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = keyboardctr.keyboard_controller:main'
        ],
    },
)
