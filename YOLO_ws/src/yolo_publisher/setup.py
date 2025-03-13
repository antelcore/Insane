from setuptools import setup

package_name = 'yolo_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 이 부분 확인
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antel',
    maintainer_email='ati0423@hanyang.ac.kr',
    description='YOLO Publisher for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_publisher = yolo_publisher.yolo_publisher:main',
<<<<<<< HEAD
=======
            'test = yolo_publisher.test_yolo:main',
            'cuda = yolo_publisher.cuda_test:main',
            'which = yolo_publisher.which_python:main',
>>>>>>> 6e2466ff8 (Change Folder structure)
        ],
    },
)

