from setuptools import find_packages, setup

package_name = 'quick_chase_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=[
        'setuptools', 'numpy', 'scipy', 'Pillow', 'PyYAML'
    ],
    zip_safe=True,
    maintainer='Farooq Olanrewaju',
    maintainer_email='olanrewajufarooq@gmail.com',
    description='F1Tenth Competition Controller',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = quick_chase_controller.controller_node:main',
        ],
    },
)
