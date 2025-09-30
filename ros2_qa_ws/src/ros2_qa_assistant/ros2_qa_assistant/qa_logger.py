import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Optional


class QALogger:
    """统一的日志处理器，用于所有QA Assistant节点"""
    
    _instance: Optional['QALogger'] = None
    _logger: Optional[logging.Logger] = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(QALogger, cls).__new__(cls)
            cls._instance._setup_logger()
        return cls._instance
    
    def _setup_logger(self):
        # 创建logs目录在src中
        # 尝试多种方式找到workspace的src目录
        current_file = Path(__file__).resolve()
        
        # 首先尝试从环境变量或工作目录找到workspace根目录
        workspace_dir = None
        
        # 方法1: 检查是否在ROS workspace中
        cwd = Path.cwd()
        for parent in [cwd] + list(cwd.parents):
            if (parent / "src").exists() and (parent / "src" / "ros2_qa_assistant").exists():
                workspace_dir = parent
                break
        
        # 方法2: 从当前文件路径向上查找
        if workspace_dir is None:
            for parent in current_file.parents:
                if parent.name == "ros2_qa_ws" or (parent / "src").exists():
                    workspace_dir = parent
                    break
        
        # 方法3: 使用默认路径
        if workspace_dir is None:
            workspace_dir = Path("/home/xiu-qi/Desktop/ros/ros2_qa_ws")
        
        log_dir = workspace_dir / "src" / "logs"
        log_dir.mkdir(exist_ok=True)
        
        # 创建唯一的日志文件名（基于日期和时间）
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = log_dir / f"qa_assistant_{timestamp}.log"
        
        # 设置日志配置
        self._logger = logging.getLogger('qa_assistant')
        self._logger.setLevel(logging.INFO)
        
        # 清除现有的处理器
        for handler in self._logger.handlers[:]:
            self._logger.removeHandler(handler)
        
        # 创建文件处理器
        file_handler = logging.FileHandler(log_file, mode='a', encoding='utf-8')
        file_handler.setLevel(logging.INFO)
        
        # 创建控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        
        # 创建格式器
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - [%(levelname)s] - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        # 添加处理器到日志器
        self._logger.addHandler(file_handler)
        self._logger.addHandler(console_handler)
        
        # 防止重复日志
        self._logger.propagate = False
    
    def log_node_start(self, node_name: str, details: str = ""):
        message = f"Node '{node_name}' started"
        if details:
            message += f" - {details}"
        self._logger.info(message)
    
    def log_node_stop(self, node_name: str):
        self._logger.info(f"Node '{node_name}' stopped")
    
    def log_message_received(self, node_name: str, topic: str, message: str):
        self._logger.info(f"[{node_name}] Received on '{topic}': {message}")
    
    def log_message_published(self, node_name: str, topic: str, message: str):
        self._logger.info(f"[{node_name}] Published to '{topic}': {message}")
    
    def log_service_call(self, node_name: str, service: str, request: str = "", response: str = ""):
        message = f"[{node_name}] Service call '{service}'"
        if request:
            message += f" - Request: {request}"
        if response:
            message += f" - Response: {response}"
        self._logger.info(message)
    
    def log_error(self, node_name: str, error_msg: str):
        self._logger.error(f"[{node_name}] ERROR: {error_msg}")
    
    def log_warning(self, node_name: str, warning_msg: str):
        self._logger.warning(f"[{node_name}] WARNING: {warning_msg}")
    
    def log_info(self, node_name: str, info_msg: str):
        self._logger.info(f"[{node_name}] {info_msg}")


def get_qa_logger() -> QALogger:
    return QALogger()