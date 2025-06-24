from setuptools import setup

package_name = 'scout_one'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='ROS2 Python package for driving a robot in a square pattern',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
      'console_scripts': [
       'square_drive = scout_one.square_drive_node:main',
      ],
    },
)

entry_points={
    'console_scripts': [
        'square_drive = scout_one.square_drive_node:main',
    ],
},