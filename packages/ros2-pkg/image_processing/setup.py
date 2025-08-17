from setuptools import find_packages, setup

package_name = 'image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='Kesler',
    maintainer_email='chuangtsh0526@gmail.com',
    description='subscribe to image topic and process the photo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_coffee_photo = image_processing.get_coffee_photo:main'
        ],
    },
)
