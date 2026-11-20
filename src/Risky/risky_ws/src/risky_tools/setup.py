from glob import glob

from setuptools import setup

package_name = 'risky_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo Risky',
    maintainer_email='mecatronica.ia14@gmail.com',
    description='Nodos de apoyo de Risky: teleoperación por teclado, adaptadores de simulación y puente de la cámara.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = risky_tools.keyboard_teleop:main',
            'thrust_adapter = risky_tools.thrust_adapter:main',
            'camera_bridge = risky_tools.camera_bridge:main',
            'dashboard = risky_tools.dashboard:main',
        ],
    },
)
