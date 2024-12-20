from setuptools import setup

package_name = 'bumperbot_py_examples'

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
    maintainer='mhered',
    maintainer_email='manolo.heredia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = bumperbot_py_examples.simple_publisher:main',
            'simple_subscriber = bumperbot_py_examples.simple_subscriber:main',
            'simple_parametric = bumperbot_py_examples.simple_parametric:main',
            'simple_turtlesim_kinematics = bumperbot_py_examples.simple_turtlesim_kinematics:main',
            'simple_tf_kinematics = bumperbot_py_examples.simple_tf_kinematics:main',
        ],
    },
)
