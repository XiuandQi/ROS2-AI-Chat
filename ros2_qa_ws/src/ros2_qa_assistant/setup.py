from setuptools import setup

package_name = 'ros2_qa_assistant'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/qa_assistant_launch.py']),
        ('share/' + package_name + '/config', [
            'config/knowledge_base.json',
            'config/rosbridge_qos.yaml'
        ]),
        ('share/' + package_name + '/web', [
            'web/index.html',
            'web/roslib.min.js',
            'web/ros2_qa.js',
            'web/style.css',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XiuandQi',
    maintainer_email='maintainer@example.com',
    description='ROS2 Rolling 教育机器人问答系统：Web交互、本地知识库、外部API拓展、RViz 可视化、多节点架构。',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_input_node = ros2_qa_assistant.web_input_node:main',
            'qa_core_node = ros2_qa_assistant.qa_core_node:main',
            'knowledge_base_server = ros2_qa_assistant.knowledge_base_server:main',
            'output_manager_node = ros2_qa_assistant.output_manager_node:main',
        ],
    },
)
