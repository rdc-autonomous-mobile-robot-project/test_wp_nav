from setuptools import setup
from glob import glob

package_name = 'test_wp_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('lib/' + package_name, glob('test_wp_nav/*.py')),
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuki',
    maintainer_email='s19c1057fa@s.chibakoudai.jp',
    description='ros2 package for waypoint navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_wp_nav = test_wp_nav.run:main'
        ],
    },
)