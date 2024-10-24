from setuptools import find_packages, setup

package_name = 'micro_mouse_controller'

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
    maintainer='chandansinghchauhan',
    maintainer_email='chandan22102@iiitnr.edu.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "maze_controller=micro_mouse_controller.maze_solver:main"
        ],
    },
)