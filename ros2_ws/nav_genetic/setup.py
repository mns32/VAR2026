from setuptools import setup

package_name = 'nav_genetic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marina',
    maintainer_email='marina@ua.es',
    description='Wall-follower del Turtlebot3 con parametros optimizados por GA.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'wall_follower = nav_genetic.controller:main',
            'ga_train      = nav_genetic.ga_train:main',
            'run_best      = nav_genetic.run_best:main',
        ],
    },
)
