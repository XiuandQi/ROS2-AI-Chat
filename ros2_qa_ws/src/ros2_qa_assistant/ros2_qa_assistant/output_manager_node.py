import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from .qa_logger import get_qa_logger


class OutputManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('output_manager_node')
        self.qa_logger = get_qa_logger()
        
        self.declare_parameter('answer_topic', '/answer')
        self.declare_parameter('web_answer_topic', '/web_answer_output')
        self.declare_parameter('viz_topic', '/qa_visualization')

        answer_topic = self.get_parameter('answer_topic').get_parameter_value().string_value
        web_answer_topic = self.get_parameter('web_answer_topic').get_parameter_value().string_value
        viz_topic = self.get_parameter('viz_topic').get_parameter_value().string_value

        self.web_pub_ = self.create_publisher(String, web_answer_topic, 10)
        self.marker_pub_ = self.create_publisher(Marker, viz_topic, 10)
        self.create_subscription(String, answer_topic, self._on_answer, 10)

        self.get_logger().info(
            f'输出管理节点启动完成 - 订阅: {answer_topic}, Web发布: {web_answer_topic}, RViz: {viz_topic}'
        )
        self.qa_logger.log_node_start(
            'output_manager_node',
            f'Subscribing to {answer_topic}, Publishing to {web_answer_topic} and {viz_topic}'
        )

    def _on_answer(self, msg: String) -> None:
        text = msg.data
        self.qa_logger.log_message_received('output_manager_node', '/answer', text)
        
        # 转发到Web界面
        self.web_pub_.publish(msg)
        self.qa_logger.log_message_published('output_manager_node', '/web_answer_output', text)

        # 发布RViz文本标记
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.3  
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.text = text
        self.marker_pub_.publish(marker)
        self.get_logger().info('已发布答案到Web和RViz')
        self.qa_logger.log_info('output_manager_node', 'Published answer to web and RViz marker')


def main(args=None):
    rclpy.init(args=args)
    node = OutputManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.qa_logger.log_info('output_manager_node', 'Received keyboard interrupt')
    finally:
        node.qa_logger.log_node_stop('output_manager_node')
        node.destroy_node()
        rclpy.shutdown()
