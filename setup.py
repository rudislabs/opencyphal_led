from setuptools import setup

package_name = 'opencyphal_led'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Benjamin Perseghetti',
    author_email='info@rudislabs.com',
    maintainer='Benjamin Perseghetti',
    maintainer_email='info@rudislabs.com',
    description='ROS 2 node for sending opencyphal led messages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencyphal_led_node = opencyphal_led.opencyphal_led_node:main'
        ],
    },
)
