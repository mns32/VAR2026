from setuptools import setup

package_name = 'turtlebot_gazebo_race'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='User',
    author_email='user@todo.todo',
    description='Turtlebot3 race con Q-Learning',
    license='BSD',
    entry_points={},
)
