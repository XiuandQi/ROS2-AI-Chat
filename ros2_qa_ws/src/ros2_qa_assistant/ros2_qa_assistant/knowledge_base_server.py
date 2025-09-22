import json
from pathlib import Path
from typing import Dict, Optional
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
from .qa_logger import get_qa_logger


class KnowledgeBaseServer(Node):
    def __init__(self) -> None:
        super().__init__('knowledge_base_server')
        self.qa_logger = get_qa_logger()
        
        # 参数配置
        self.declare_parameter('service_name', '/query_knowledge_base')
        self.declare_parameter('knowledge_base_path', 'config/knowledge_base.json')
        self.declare_parameter('question_topic', '/question')
        self.declare_parameter('kb_query_topic', '/kb_query')
        self._service_name = self.get_parameter('service_name').get_parameter_value().string_value
        kb_path_param = self.get_parameter('knowledge_base_path').get_parameter_value().string_value
        self._kb_path = self._resolve_path(kb_path_param)

        self._kb_entries = []  # 存储知识库条目列表
        self._last_question = None
        self._load_kb()

        self.srv_ = self.create_service(SetBool, self._service_name, self._handle_query)
        # 订阅问题话题缓存最新问题
        question_topic = self.get_parameter('question_topic').get_parameter_value().string_value
        kb_query_topic = self.get_parameter('kb_query_topic').get_parameter_value().string_value
        self.create_subscription(String, question_topic, self._on_question, 10)
        # 内部查询话题订阅
        if kb_query_topic != question_topic:
            self.create_subscription(String, kb_query_topic, self._on_question, 10)
        self.get_logger().info(
            f'知识库服务器启动完成 - 服务: {self._service_name}, 知识库: {self._kb_path}'
        )
        self.qa_logger.log_node_start(
            'knowledge_base_server',
            f'Service: {self._service_name}, KB path: {self._kb_path}, Subscribed to: {question_topic}'
        )

    def _on_question(self, msg: String) -> None:
        self._last_question = msg.data.strip()
        self.get_logger().debug(f'缓存问题: {self._last_question}')
        self.qa_logger.log_info('knowledge_base_server', f'Cached question: {self._last_question}')

    def _resolve_path(self, path_str: str) -> Path:
        p = Path(path_str)
        if p.is_absolute() and p.exists():
            return p
        # 尝试ament包目录
        try:
            from ament_index_python.packages import get_package_share_directory  # type: ignore
            share_dir = Path(get_package_share_directory('ros2_qa_assistant'))
            candidate = share_dir / p
            if candidate.exists():
                return candidate
        except Exception:
            pass
        # 尝试源码包目录
        pkg_src_dir = Path(__file__).resolve().parents[1]
        candidate2 = pkg_src_dir / p
        if candidate2.exists():
            return candidate2
        # 尝试当前工作目录
        candidate3 = Path.cwd() / p
        if candidate3.exists():
            return candidate3
        # 使用绝对路径
        return (Path.cwd() / p).resolve()

    def _find_best_answer(self, question: str) -> Optional[str]:
        """基于关键词匹配和每个关键词的权重找到最佳答案"""
        if not question or not self._kb_entries:
            return None
            
        normalized_question = self._normalize(question)
        question_words = set(normalized_question.split())
        
        best_score = 0.0
        best_answer = None
        
        for entry in self._kb_entries:
            total_score = 0.0
            matched_keywords = 0
            
            # 计算每个关键词的匹配分数
            for keyword_item in entry['keywords']:
                keyword = keyword_item['word']
                weight = keyword_item['weight']
                
                if keyword in normalized_question:
                    # 完全匹配关键词
                    total_score += weight * 2.0
                    matched_keywords += 1
                else:
                    # 检查关键词的单词是否在问题中
                    keyword_words = set(keyword.split())
                    overlap = len(keyword_words.intersection(question_words))
                    if overlap > 0:
                        # 部分匹配，按重叠比例和权重计分
                        overlap_ratio = overlap / len(keyword_words)
                        partial_score = overlap_ratio * weight
                        total_score += partial_score
                        if overlap_ratio > 0.5:  # 大于50%匹配才算匹配的关键词
                            matched_keywords += 1
            
            # 只有至少匹配一个关键词才考虑
            if matched_keywords > 0 and total_score > best_score:
                best_score = total_score
                best_answer = entry['answer']
        
        return best_answer

    def _normalize(self, text: str) -> str:
        # 文本标准化：去除空格、转小写、去标点
        s = str(text).strip().lower()
        strip_chars = '？?。.!！,，;；:\n\t\r '
        s = s.strip(strip_chars)
        return s

    def _load_kb(self) -> None:
        try:
            with open(self._kb_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if not isinstance(data, list):
                raise ValueError('知识库文件必须是包含{keywords, answer}对象的JSON数组')
            
            self._kb_entries = []
            for item in data:
                if not isinstance(item, dict):
                    continue
                if 'keywords' not in item or 'answer' not in item:
                    continue
                    
                # 验证并标准化条目
                keywords = item['keywords']
                if not isinstance(keywords, list):
                    continue
                
                answer = str(item['answer']).strip()
                if not answer:
                    continue
                
                # 处理关键词列表，支持每个关键词有独立权重
                processed_keywords = []
                for kw in keywords:
                    if isinstance(kw, dict) and 'word' in kw:
                        # 新格式：{"word": "关键词", "weight": 权重}
                        word = str(kw['word']).strip()
                        weight = float(kw.get('weight', 1.0))
                        if word:
                            processed_keywords.append({
                                'word': self._normalize(word),
                                'weight': weight
                            })
                    elif isinstance(kw, str):
                        # 兼容旧格式：直接是字符串，默认权重1.0
                        word = str(kw).strip()
                        if word:
                            processed_keywords.append({
                                'word': self._normalize(word),
                                'weight': 1.0
                            })
                
                if processed_keywords:
                    self._kb_entries.append({
                        'keywords': processed_keywords,
                        'answer': answer
                    })
            
            self.get_logger().info(f'已加载 {len(self._kb_entries)} 个知识库条目')
            self.qa_logger.log_info('knowledge_base_server', f'Successfully loaded {len(self._kb_entries)} KB entries from {self._kb_path}')
        except Exception as e:
            self.get_logger().error(f'加载知识库失败 {self._kb_path}: {e}')
            self.qa_logger.log_error('knowledge_base_server', f'Failed to load knowledge base from {self._kb_path}: {e}')
            self._kb_entries = []

    def _handle_query(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        # 使用最后接收到的问题进行查询
        question = (self._last_question or '').strip()
        self.qa_logger.log_service_call('knowledge_base_server', '/query_knowledge_base', f'Query for: {question}')
        
        # 使用新的关键词匹配算法
        answer = self._find_best_answer(question)
        
        if answer is None:
            response.success = False
            response.message = '未在本地知识库中找到答案'
            self.qa_logger.log_service_call('knowledge_base_server', '/query_knowledge_base', '', 'Miss - not found in KB')
        else:
            response.success = True
            response.message = answer
            self.qa_logger.log_service_call('knowledge_base_server', '/query_knowledge_base', '', f'Hit - found answer: {answer}')
        
        self.get_logger().info(f'知识库查询 "{question}" -> {"命中" if response.success else "未找到"}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = KnowledgeBaseServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.qa_logger.log_info('knowledge_base_server', 'Received keyboard interrupt')
    finally:
        node.qa_logger.log_node_stop('knowledge_base_server')
        node.destroy_node()
        rclpy.shutdown()
