from typing import Optional, List, Dict
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .qa_logger import get_qa_logger


class QACoreNode(Node):
    def __init__(self) -> None:
        super().__init__('qa_core_node')
        self.qa_logger = get_qa_logger()
        
        self.conversation_history: List[Dict[str, str]] = []
        self.max_history_length = 10
        
        self.declare_parameter('question_topic', '/question')
        self.declare_parameter('answer_topic', '/answer')
        self.declare_parameter('kb_service', '/query_knowledge_base')
        self.declare_parameter('kb_query_topic', '/kb_query')
        self.declare_parameter('enable_context', True)
        self.declare_parameter('max_history_length', 10)

        question_topic = self.get_parameter('question_topic').get_parameter_value().string_value
        answer_topic = self.get_parameter('answer_topic').get_parameter_value().string_value
        kb_service = self.get_parameter('kb_service').get_parameter_value().string_value
        kb_query_topic = self.get_parameter('kb_query_topic').get_parameter_value().string_value
        self.enable_context = self.get_parameter('enable_context').get_parameter_value().bool_value
        self.max_history_length = self.get_parameter('max_history_length').get_parameter_value().integer_value

        self.answer_pub_ = self.create_publisher(String, answer_topic, 10)
        self.create_subscription(String, question_topic, self._on_question, 10)
        self.kb_query_pub_ = self.create_publisher(String, kb_query_topic, 10)
        self.kb_client_ = self.create_client(SetBool, kb_service)
        self.create_service(SetBool, '/clear_context', self._clear_context_callback)

        self.qa_logger.log_node_start(
            'qa_core_node', 
            f'Subscribed to {question_topic}, Publishing to {answer_topic}, KB Service: {kb_service}, Context: {self.enable_context}'
        )

    def _clear_context_callback(self, request, response):
        self.conversation_history.clear()
        response.success = True
        response.message = '对话历史已清除'
        self.qa_logger.log_info('qa_core_node', 'Conversation history cleared')
        return response

    def _add_to_history(self, role: str, content: str) -> None:
        if not self.enable_context:
            return
        
        self.conversation_history.append({
            "role": role,
            "content": content
        })
        
        while len(self.conversation_history) > self.max_history_length * 2:
            for i, msg in enumerate(self.conversation_history):
                if msg["role"] != "system":
                    self.conversation_history.pop(i)
                    break

    def _on_question(self, msg: String) -> None:
        question = msg.data.strip()
        if not question:
            return
        self.qa_logger.log_message_received('qa_core_node', '/question', question)
        
        self._add_to_history("user", question)
        
        try:
            m = String()
            m.data = question
            self.kb_query_pub_.publish(m)
            self.qa_logger.log_message_published('qa_core_node', '/kb_query', question)
        except Exception as e:
            self.qa_logger.log_error('qa_core_node', f'Failed to publish to KB query topic: {e}')
            
        # 等待知识库服务
        if not self.kb_client_.wait_for_service(timeout_sec=2.0):
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
                self.qa_logger.log_error('qa_core_node', f'KB query failed: {e}')
            if not ans:
                ans = self.query_external_api(q)
            if not ans:
                ans = '抱歉，我现在无法回答这个问题。'
            
            self._add_to_history("assistant", ans)
            
            out = String()
            out.data = ans
            self.answer_pub_.publish(out)
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
        
        self._add_to_history("assistant", ans)
        
        out = String()
        out.data = ans
        self.answer_pub_.publish(out)
        self.qa_logger.log_message_published('qa_core_node', '/answer', ans)

    def query_external_api(self, question: str) -> Optional[str]:
        self.url = 'https://spark-api-open.xf-yun.com/v2/chat/completions'
        
        messages = [
            {
                "role": "system",
                "content": "你是一个智能助手，能够根据上下文进行连贯的对话。"
            }
        ]
        
        if self.enable_context and self.conversation_history:
            messages.extend(self.conversation_history)
        else:
            messages.append({
                "role": "user",
                "content": question
            })
        
        self.data = {
            "max_tokens" : 32768,
            "top_k" : 6,
            "temperature" : 1.2,
            "messages" : messages,
            "model" : "x1",
            "tools" : [
                {
                    "web_search" : {
                        "search_mode" : "normal",
                        "enable" : False
                    },
                    "type" : "web_search"
                }
            ]
        }
        self.data["stream"] = False
        self.header = {
            "Authorization": "Bearer iFUlrMnNwaCqtmiQAMBj:BqdmYdEiejRZzfrwMXGD"
        }
        self.response = requests.post(url=self.url, headers=self.header, json=self.data, stream=False)
        self.response.encoding = 'utf-8'
        self.text = self.response.json().get('choices')[0]['message']['content']
        return self.text if self.text else None


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
