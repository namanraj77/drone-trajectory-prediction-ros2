from setuptools import setup
import os
from glob import glob

package_name = 'drone_predictor_clean'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    # Package index
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),

    # Package manifest
    ('share/' + package_name, ['package.xml']),

    # Launch files
    (os.path.join('share', package_name, 'launch'),
     glob('launch/*.launch.py')),

    # Model files
    (os.path.join('share', package_name, 'models'),
     glob('models/*.pt')),

    # RViz config
    (os.path.join('share', package_name, 'rviz'),
     glob('rviz/*.rviz')),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tywin',
    maintainer_email='tywin@todo.todo',
    description='ML-based drone trajectory predictor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'predictor_node = drone_predictor_clean.predictor_node:main',
        'pose_publisher = drone_predictor_clean.pose_publisher:main',
    ],
},

)
