from setuptools import setup

package_name = 'cartesian_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranesh',
    maintainer_email='pranesh@todo.todo',
    description='Keyboard x/y/z straight-line jogging through MoveIt',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartesian_keys = cartesian_teleop.cartesian_keys:main',
        ],
    },
)
