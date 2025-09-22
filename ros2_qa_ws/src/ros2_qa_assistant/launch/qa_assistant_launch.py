from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='ros2_qa_assistant',
            executable='knowledge_base_server',
            name='knowledge_base_server',
            output='screen',
            parameters=[{
                'service_name': '/query_knowledge_base',
                'knowledge_base_path': 'config/knowledge_base.json',
            }],
        ),
        Node(
            package='ros2_qa_assistant',
            executable='qa_core_node',
            name='qa_core_node',
            output='screen',
            parameters=[{
                'question_topic': '/question',
                'answer_topic': '/answer',
                'kb_service': '/query_knowledge_base',
            }],
        ),
        Node(
            package='ros2_qa_assistant',
            executable='web_input_node',
            name='web_input_node',
            output='screen',
            parameters=[{
                'web_input_topic': '/web_question_input',
                'question_topic': '/question',
            }],
        ),
        Node(
            package='ros2_qa_assistant',
            executable='output_manager_node',
            name='output_manager_node',
            output='screen',
            parameters=[{
                'answer_topic': '/answer',
                'web_answer_topic': '/web_answer_output',
                'viz_topic': '/qa_visualization',
            }],
        ),
    ])
