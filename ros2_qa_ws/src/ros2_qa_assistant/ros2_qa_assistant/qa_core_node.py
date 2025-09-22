from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .qa_logger import get_qa_logger


class QACoreNode(Node):
    def __init__(self) -> None:
        super().__init__('qa_core_node')
        self.qa_logger = get_qa_logger()
        
        self.declare_parameter('question_topic', '/question')
        self.declare_parameter('answer_topic', '/answer')
        self.declare_parameter('kb_service', '/query_knowledge_base')
        self.declare_parameter('kb_query_topic', '/kb_query')

        question_topic = self.get_parameter('question_topic').get_parameter_value().string_value
        answer_topic = self.get_parameter('answer_topic').get_parameter_value().string_value
        kb_service = self.get_parameter('kb_service').get_parameter_value().string_value
        kb_query_topic = self.get_parameter('kb_query_topic').get_parameter_value().string_value

        self.answer_pub_ = self.create_publisher(String, answer_topic, 10)
        self.create_subscription(String, question_topic, self._on_question, 10)
        # 发布到内部查询话题
        self.kb_query_pub_ = self.create_publisher(String, kb_query_topic, 10)
        self.kb_client_ = self.create_client(SetBool, kb_service)

        self.get_logger().info(
            f'QA核心节点启动完成 - 订阅: {question_topic}, 发布: {answer_topic}, 知识库服务: {kb_service}'
        )
        self.qa_logger.log_node_start(
            'qa_core_node', 
            f'Subscribed to {question_topic}, Publishing to {answer_topic}, KB Service: {kb_service}'
        )

    def _on_question(self, msg: String) -> None:
        question = msg.data.strip()
        if not question:
            return
        self.get_logger().info(f'收到问题: {question}')
        self.qa_logger.log_message_received('qa_core_node', '/question', question)
        
        # 发布到内部查询话题传递问题文本
        try:
            m = String()
            m.data = question
            self.kb_query_pub_.publish(m)
            self.qa_logger.log_message_published('qa_core_node', '/kb_query', question)
        except Exception as e:
            self.qa_logger.log_error('qa_core_node', f'Failed to publish to KB query topic: {e}')
            
        # 等待知识库服务
        if not self.kb_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('知识库服务不可用')
            self.qa_logger.log_warning('qa_core_node', 'Knowledge base service unavailable')
            self._publish_fallback_answer(question)
            return
        req = SetBool.Request()
        req.data = True
        future = self.kb_client_.call_async(req)
        self.qa_logger.log_service_call('qa_core_node', '/query_knowledge_base', 'Query for knowledge base')

        def _on_done(fut, q=question):
            ans: Optional[str] = None
            try:
                resp = fut.result()
                if resp and resp.success:
                    ans = resp.message
                    self.qa_logger.log_service_call('qa_core_node', '/query_knowledge_base', '', f'Success: {ans}')
                else:
                    self.qa_logger.log_service_call('qa_core_node', '/query_knowledge_base', '', 'Failed or no success')
            except Exception as e:
                self.get_logger().error(f'知识库查询失败: {e}')
                self.qa_logger.log_error('qa_core_node', f'KB query failed: {e}')
            if not ans:
                ans = self.query_external_api(q)
            if not ans:
                ans = '抱歉，我现在无法回答这个问题。'
            out = String()
            out.data = ans
            self.answer_pub_.publish(out)
            self.get_logger().info(f'发布答案: {ans}')
            self.qa_logger.log_message_published('qa_core_node', '/answer', ans)

        future.add_done_callback(_on_done)

    def query_knowledge_base(self, question: str) -> Optional[str]:
        # 同步查询方法（保留但不在回调中使用）
        try:
            if not self.kb_client_.wait_for_service(timeout_sec=2.0):
                return None
            req = SetBool.Request()
            req.data = True
            future = self.kb_client_.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if not future.done():
                return None
            resp = future.result()
            if resp and resp.success:
                return resp.message
            return None
        except Exception:
            return None

    def _publish_fallback_answer(self, question: str) -> None:
        ans = self.query_external_api(question)
        if not ans:
            ans = '抱歉，我现在无法回答这个问题。'
        out = String()
        out.data = ans
        self.answer_pub_.publish(out)
        self.get_logger().info(f'发布答案: {ans}')
        self.qa_logger.log_message_published('qa_core_node', '/answer', ans)

    def query_external_api(self, question: str) -> Optional[str]:
        # 外部API接口（暂未配置）
        self.get_logger().info('未配置外部API，跳过')
        self.qa_logger.log_info('qa_core_node', 'No external API configured. Skipping external query')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = QACoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.qa_logger.log_info('qa_core_node', 'Received keyboard interrupt')
    finally:
        node.qa_logger.log_node_stop('qa_core_node')
        node.destroy_node()
        rclpy.shutdown()
