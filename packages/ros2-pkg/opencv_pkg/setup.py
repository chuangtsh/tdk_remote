from setuptools import find_packages, setup

package_name = 'opencv_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['opencv_ros', 'opencv_ros.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'coffee_sticker_gui = opencv_ros.coffee_sticker_gui:main',
            'coffee_sticker = opencv_ros.coffee_sticker:main',
            'coffee_menu_gui = opencv_ros.coffee_menu_gui:main',
            'coffee_menu = opencv_ros.coffee_menu:main'
        ],
    },
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='CYH',
    description='TDK camera opencv example',
)