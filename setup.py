from setuptools import setup

package_name = 'simple_rover_simulation'

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
    maintainer='Miro Voellmy',
    maintainer_email='miro.voellmy@esa.int',
    description='This package simply sends the joint states back as requested in the rover_joint_cmds.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_rover_simulation_node = simple_rover_simulation.simple_rover_simulation_node:main'
        ],
    },
)
