from setuptools import setup

package_name = 'map_to_pic'

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
    maintainer='rodion',
    maintainer_email='rodion_anisimov@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_to_pic=map_to_pic.map_to_pic:main',
            'lifecycle_caller=map_to_pic.lifecycle_caller:main',
            'nav_action=map_to_pic.nav_action:main',
            'run_sh=map_to_pic.run_sh:main',
        ],
    },
)
