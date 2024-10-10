from setuptools import find_packages, setup

package_name = 'dangerous_areas_solution'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Scott Howsam',
    maintainer_email='u7115241@anu.edu.au',
    description='Dangerous areas implementation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dangerous_areas_solution_node = dangerous_areas_solution.dangerous_areas_solution_node:main'
        ],
    },
)
