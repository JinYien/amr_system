from glob import glob

from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/map', glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JinYien',
    maintainer_email='leejinyien@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
)
