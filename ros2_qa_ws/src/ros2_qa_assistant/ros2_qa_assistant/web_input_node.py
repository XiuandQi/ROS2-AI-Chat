import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from .qa_logger import get_qa_logger


class WebInputNode(Node):
    def __init__(self) -> None:
        super().__init__('web_input_node')
        self.qa_logger = get_qa_logger()
        
        self.declare_parameter('web_input_topic', '/web_question_input')
        self.declare_parameter('question_topic', '/question')

        web_input_topic = self.get_parameter('web_input_topic').get_parameter_value().string_value
        question_topic = self.get_parameter('question_topic').get_parameter_value().string_value

        # QoS配置与rosbridge兼容
        web_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(String, question_topic, 10)
        self.subscription_ = self.create_subscription(String, web_input_topic, self._on_web_input, web_qos)
        self.qa_logger.log_node_start(
            'web_input_node',
            f'Subscribing to {web_input_topic}, Publishing to {question_topic}'
        )

    def _on_web_input(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            self.qa_logger.log_warning('web_input_node', 'Received empty question from web, ignoring')
            return
        
        self.qa_logger.log_message_received('web_input_node', '/web_question_input', text)
        
        out = String()
        out.data = text
        self.publisher_.publish(out)
        self.qa_logger.log_message_published('web_input_node', '/question', text)


def main(args=None):
    rclpy.init(args=args)
    node = WebInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.qa_logger.log_info('web_input_node', 'Received keyboard interrupt')
    finally:
        node.qa_logger.log_node_stop('web_input_node')
        node.destroy_node()
        rclpy.shutdown()
