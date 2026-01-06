from setuptools import setup, find_packages

package_name = 'server_gateway'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi>=0.104.0',
        'uvicorn>=0.24.0',
        'websockets>=12.0',
        'paho-mqtt>=1.6.0',
    ],
    zip_safe=True,
    maintainer='Hasan Çoban',
    maintainer_email='hasancoban@std.iyte.edu.tr',
    description='Server gateway for hand gesture control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_gateway = server_gateway.main:main',
        ],
    },
)

