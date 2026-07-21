from setuptools import find_packages, setup
import sys

# colcon (ament_python) may pass legacy develop flags that are not accepted by
# some setuptools versions. Strip them for compatibility.
for _flag in ("--editable", "--uninstall"):
    while _flag in sys.argv:
        sys.argv.remove(_flag)

while "--build-directory" in sys.argv:
    idx = sys.argv.index("--build-directory")
    del sys.argv[idx]
    if idx < len(sys.argv):
        del sys.argv[idx]

package_name = 'op3_football'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/joints.yaml', 'config/l3_motion.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotis',
    maintainer_email='robotis@todo.todo',
    description='OP3 football stack L2/L3/L4 Python API',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'l3_smoke = op3_football.scripts.l3_smoke:main',
            'l4_stub = op3_football.l4.football:main',
            'dualsense_teleop = op3_football.scripts.dualsense_teleop:main',
        ],
    },
)
