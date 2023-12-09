from setuptools import find_packages, setup

package_name = 'nubot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manual_explore.launch.xml','launch/explore.launch.xml']),
        ('share/' + package_name + '/config', ['config/bot_params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdalal',
    maintainer_email='shaildalal2024@u.northwestern.edu',
    description='Navigation package for the nubot diff drive robot for mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "explore=nubot_nav.explore:main"
        ],
    },
)
