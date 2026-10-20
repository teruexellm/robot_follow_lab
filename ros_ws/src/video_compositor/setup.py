from setuptools import setup

package_name = 'video_compositor'

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
    description='combined.mp4 の生成 (spec v2.0 §57-§59)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frame_grabber = video_compositor.frame_grabber_node:main',
            'compose_video = video_compositor.compose_video:main',
        ],
    },
)
