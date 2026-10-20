from setuptools import setup

package_name = 'follower_tracking'

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
    description='Target Pose 推定と追従制御',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_pose_estimator = follower_tracking.target_pose_estimator:main',
            'follow_controller = follower_tracking.follow_controller:main',
        ],
    },
)
