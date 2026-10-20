from setuptools import setup

package_name = 'experiment_logger'

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
    description='telemetry / events / summary の記録 (spec v2.0 §55, §56, §60)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'experiment_logger = experiment_logger.experiment_logger_node:main',
        ],
    },
)
