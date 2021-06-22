from setuptools import setup

package_name = 'tf_servo_pkg'

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
    maintainer='snortman',
    maintainer_email='scott.nortman@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_servo = tf_servo_pkg.tf_servo:main',
            'tf_spacenav = tf_servo_pkg.tf_servo_spacenav:main'
        ],
    },
)
