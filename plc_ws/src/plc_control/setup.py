from setuptools import setup
import os
from glob import glob

package_name = 'plc_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tobias',
    maintainer_email='93526045+TobiasBJensen@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "plc_node = plc_control.plc_node_script:main",
            "display_node = plc_control.display_node_script:main",
            "control_node = plc_control.control_node_script:main"
        ],
    },
)
