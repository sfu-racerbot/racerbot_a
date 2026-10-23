from setuptools import setup

package_name = 'waypoint_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'numpy>=1.23', 'scipy>=1.10',
                      'scikit-image>=0.24', 'Pillow>=9', 'PyYAML>=6',
                      'shapely>=1.8.5'],
    extras_require={'test': ['pytest>=7']},
    python_requires='>=3.10',
    zip_safe=True,
    maintainer='SFU Racerbot',
    maintainer_email='sfuracerbot@gmail.com',
    description='Offline map-to-centerline CSV generation with footprint validation.',
    license='MIT',
    entry_points={'console_scripts': [
        'generate_waypoints = waypoint_generator.cli:main',
    ]},
)
