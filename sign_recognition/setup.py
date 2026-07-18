from setuptools import find_packages, setup

package_name = 'sign_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='sunrise@example.com',
    description='Sign recognition and capture package for racing car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sign_capture_node = sign_recognition.sign_capture_node:main',
        ],
    },
)
