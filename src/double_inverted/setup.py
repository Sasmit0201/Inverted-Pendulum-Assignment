from setuptools import find_packages, setup

package_name = 'double_inverted'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/double_inverted_pendulum.launch.py']),
        ('share/' + package_name + '/urdf',
            ['urdf/double_pendulum.urdf.xacro']),
        ('share/' + package_name + '/config',
            ['config/params.yaml', 'config/eval_thresholds.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='you@example.com',
    description='Double inverted pendulum: sim, PID/MPC controllers, evaluator.',
    license='MIT',
    tests_require=['pytest'],
    # evaluate.py is installed as a raw executable script (ros2 run
    # double_inverted evaluate.py ...), NOT a console_scripts entry point,
    # so its location stays exactly at scripts/evaluate.py per the repo
    # layout in README.md Sec 2.
    scripts=['scripts/evaluate.py'],
    entry_points={
        'console_scripts': [
            'sim_node = double_inverted.sim_node:main',
            'controller_pid = double_inverted.controller_pid:main',
            'controller_mpc = double_inverted.controller_mpc:main',
        ],
    },
)
