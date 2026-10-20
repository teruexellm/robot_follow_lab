from setuptools import setup

package_name = 'leader_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teruexellm',
    maintainer_email='344744385+teruexellm@users.noreply.github.com',
    description='Leader の Waypoint 走行ミッション',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_mission = leader_mission.waypoint_mission:main',
        ],
    },
)
