from setuptools import setup

package_name = 'test_wp_nav'

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
    maintainer='kazuki',
    maintainer_email='s19c1057fa@s.chibakoudai.jp',
    description='ros2 package for waypoint navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_wp_nav = test_wp_nav.test_wp_nav:main'
        ],
    },
)