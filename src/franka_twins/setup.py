from setuptools import find_packages, setup

package_name = 'franka_twins'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/launch', ['launch/wave.launch.py']),
         ('share/' + package_name + '/launch', ['launch/cola.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sun',
    maintainer_email='1962945668@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['wave = franka_twins.wave_dual:main',
        'cola = franka_twins.cola_dual:main'
        ],
    },
    
)
