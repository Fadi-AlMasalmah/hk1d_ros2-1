from setuptools import find_packages, setup

package_name = 'identification_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fadi',
    maintainer_email='fadi.almasalmah94@gmail.com',
    description='a package that sends position reference to the hk1d robot for identification',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'identification_node = identification_node.identification_node:main',
            'ref_pos_publisher = identification_node.ref_pos_publisher:main',
        ],
    },
)
