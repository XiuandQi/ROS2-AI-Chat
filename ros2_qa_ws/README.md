# ROS2 QA Assistant System

基于 ROS 2 Rolling 的分布式问答系统，通过 WebSocket 提供 Web 交互界面，集成本地知识库查询、统一日志管理和可视化功能。

## 系统架构

```
┌─────────────────┐    WebSocket    ┌──────────────────┐
│   Web Interface │ ←──────────────→ │   rosbridge      │
│   (JavaScript)  │    ws://9090    │   WebSocket      │
└─────────────────┘                 └──────────────────┘
                                              │
                                              │ ROS2 Topics
                                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS2 Network                               │
├─────────────────┬─────────────────┬─────────────────┬───────────────┤
│ web_input_node  │   qa_core_node  │output_manager   │knowledge_base │
│                 │                 │    _node        │   _server     │
│ /web_question   │    /question    │   /answer       │   Service:    │
│    _input       │    /kb_query    │   /web_answer   │ /query_kb     │
│       │         │       │         │     _output     │      │        │
│       ▼         │       ▼         │       ▲         │      ▲        │
│   转发问题       │   处理问题       │   转发答案       │   查询知识库   │
└─────────────────┴─────────────────┴─────────────────┴───────────────┘
```

## 项目结构

```
ros2_qa_ws/
├── auto_build_and_run.sh          # 一键构建和运行脚本
├── src/ros2_qa_assistant/
│   ├── ros2_qa_assistant/          # 节点源码
│   │   ├── qa_core_node.py         # 核心问答处理节点
│   │   ├── web_input_node.py       # Web输入转发节点
│   │   ├── output_manager_node.py  # 输出管理节点
│   │   ├── knowledge_base_server.py# 知识库服务器
│   │   └── qa_logger.py            # 统一日志系统
│   ├── launch/
│   │   └── qa_assistant_launch.py  # 启动文件
│   ├── config/
│   │   ├── knowledge_base.json     # 本地知识库
│   │   └── rosbridge_qos.yaml      # rosbridge QoS配置
│   └── web/
│       ├── index.html              # Web界面
│       ├── ros2_qa.js              # 前端逻辑
│       ├── style.css               # 样式文件
│       └── roslib.min.js           # ROS JavaScript库
├── src/logs/                       # 日志文件目录
└── build/, install/, log/          # 构建产物
```

## 系统依赖

- Ubuntu 22.04
- ROS 2 Rolling
- Python 3.10+
- rosbridge_server (WebSocket通信)
- python3-websocket (测试工具)

安装依赖：
```bash
sudo apt update
sudo apt install ros-rolling-rosbridge-server ros-rolling-visualization-msgs python3-websocket
```

## 快速启动

使用自动脚本一键完成构建和启动：
```bash
cd /home/xiu-qi/Desktop/ros/ros2_qa_ws
./auto_build_and_run.sh
```

脚本执行流程：
1. 清理旧进程和构建文件
2. 源码环境设置 (/opt/ros/rolling)
3. 构建 ros2_qa_assistant 包
4. 启动 rosbridge_websocket (端口 9090，带QoS配置)
5. 启动四个核心节点
6. 自动打开Web界面

启动完成后访问：`http://localhost:8080/`

## 核心组件

### 1. Web输入节点 (web_input_node)
负责接收Web界面发送的问题并转发到内部处理流程。

```python
# QoS配置确保与rosbridge兼容
web_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

# 订阅来自Web的问题
self.subscription_ = self.create_subscription(
    String, '/web_question_input', self._on_web_input, web_qos
)

# 转发到内部问题话题
self.publisher_ = self.create_publisher(String, '/question', 10)
```

### 2. QA核心节点 (qa_core_node)
处理问题，调用知识库服务，管理整个问答流程。

```python
def _on_question(self, msg: String) -> None:
    # 记录收到的问题
    self.qa_logger.log_message_received('qa_core_node', '/question', msg.data)
    
    # 发布到知识库查询话题
    kb_msg = String()
    kb_msg.data = msg.data
    self.kb_query_publisher_.publish(kb_msg)
    
    # 异步调用知识库服务
    request = SetBool.Request()
    request.data = True
    future = self.kb_client_.call_async(request)
    future.add_done_callback(lambda f: self._handle_kb_response(f, msg.data))
```

### 3. 知识库服务器 (knowledge_base_server)
提供本地知识库查询服务，支持基于关键词权重的智能匹配算法。

```python
def _find_best_answer(self, question: str) -> Optional[str]:
    """基于关键词匹配和每个关键词的权重找到最佳答案"""
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
                # 完全匹配：权重 × 2.0
                total_score += weight * 2.0
                matched_keywords += 1
            else:
                # 部分匹配：重叠比例 × 权重
                keyword_words = set(keyword.split())
                overlap = len(keyword_words.intersection(question_words))
                if overlap > 0:
                    overlap_ratio = overlap / len(keyword_words)
                    total_score += overlap_ratio * weight
                    if overlap_ratio > 0.5:
                        matched_keywords += 1
        
        if matched_keywords > 0 and total_score > best_score:
            best_score = total_score
            best_answer = entry['answer']
    
    return best_answer
```

### 知识库JSON结构
知识库采用基于关键词权重的新结构：

```json
[
  {
    "keywords": [
      {"word": "ROS", "weight": 1.0},
      {"word": "机器人操作系统", "weight": 1.0},
      {"word": "Robot Operating System", "weight": 0.8}
    ],
    "answer": "ROS是机器人操作系统，提供通信、工具和库以简化机器人开发。"
  },
  {
    "keywords": [
      {"word": "话题", "weight": 1.0},
      {"word": "topic", "weight": 1.0},
      {"word": "列表", "weight": 0.7},
      {"word": "查看", "weight": 0.6}
    ],
    "answer": "使用命令：ros2 topic list"
  }
]
```

**关键词权重说明：**
- **1.0**: 核心关键词，完全匹配时优先级最高
- **0.8-0.9**: 重要相关词，中高优先级
- **0.6-0.7**: 一般相关词，辅助匹配

### 4. 输出管理节点 (output_manager_node)
管理答案输出，发送到Web界面和可视化系统。

```python
def _on_answer(self, msg: String) -> None:
    # 转发答案到Web输出话题
    self.web_pub_.publish(msg)
    
    # 创建可视化标记
    marker = self._create_qa_marker(msg.data)
    self.rviz_pub_.publish(marker)
    
    self.qa_logger.log_message_published('output_manager_node', '/web_answer_output', msg.data)
```

## 手动步骤（可选）
```bash
source /opt/ros/rolling/setup.bash
cd ~/ros2_qa_ws
colcon build --packages-select ros2_qa_assistant
source install/setup.bash
ros2 launch ros2_qa_assistant qa_assistant_launch.py
```

打开 Web：
```
file://$PWD/src/ros2_qa_assistant/web/index.html
```

## Web界面与QoS配置

### 前端JavaScript实现
```javascript
// 连接rosbridge WebSocket服务器
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

// 创建话题发布者 - 必须指定QoS以匹配ROS2节点
var questionPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/web_question_input',
    messageType: 'std_msgs/String',
    qos: {
        depth: 10,
        reliability: 'reliable',
        durability: 'transient_local'
    }
});

// 订阅答案话题
var answerSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/web_answer_output', 
    messageType: 'std_msgs/String',
    qos: {
        depth: 10,
        reliability: 'reliable',
        durability: 'transient_local'
    }
});
```

### QoS兼容性配置
为确保Web界面与ROS2节点通信，创建了专门的QoS配置文件：

```yaml
# config/rosbridge_qos.yaml
default_qos:
  depth: 10
  reliability: reliable
  durability: transient_local

/web_question_input:
  depth: 10
  reliability: reliable
  durability: transient_local

/web_answer_output:
  depth: 10
  reliability: reliable
  durability: transient_local
```

## 知识库配置与管理

### 配置文件位置
```
src/ros2_qa_assistant/config/knowledge_base.json
```

### 知识库条目结构
每个知识库条目包含关键词数组和对应答案：

```json
{
  "keywords": [
    {"word": "关键词", "weight": 权重值},
    {"word": "同义词", "weight": 权重值}
  ],
  "answer": "对应的答案内容"
}
```

### 匹配算法机制
1. **完全匹配**: 关键词完整出现在问题中
   - 得分 = 关键词权重 × 2.0
2. **部分匹配**: 关键词的部分单词出现在问题中
   - 得分 = 重叠比例 × 关键词权重
3. **最佳答案**: 选择总得分最高且至少匹配一个关键词的条目

### 权重设置建议
- **核心概念词** (1.0): ROS、节点、话题等
- **技术术语** (0.8-0.9): 具体技术名词
- **动作词** (0.6-0.7): 查看、启动、运行等
- **修饰词** (0.4-0.5): 如何、什么、怎么等

### 添加新条目示例
```json
{
  "keywords": [
    {"word": "构建", "weight": 0.9},
    {"word": "编译", "weight": 0.9},
    {"word": "build", "weight": 0.8},
    {"word": "colcon", "weight": 1.0}
  ],
  "answer": "使用 colcon build 命令构建ROS2包"
}
```

## 运行时数据流

```
用户输入 → Web界面表单 → JavaScript发布 → /web_question_input (QoS: TRANSIENT_LOCAL)
    ↓
web_input_node处理 → 转发到 /question 话题
    ↓
qa_core_node接收 → 处理问题逻辑 → 知识库查询 → knowledge_base_server服务
    ↓
答案发布 → /answer 话题 → output_manager_node转发 → /web_answer_output 话题
    ↓
Web界面接收 → 显示答案给用户
```

## 调试与监控

### 查看话题信息
```bash
# 查看话题QoS设置
ros2 topic info /web_question_input -v

# 监听话题消息
ros2 topic echo /question
ros2 topic echo /answer
```

### 日志文件分析
所有节点的运行日志统一保存在：
```
src/logs/qa_assistant_YYYYMMDD_HHMMSS.log
```

日志格式：
```
2025-09-22 15:30:45 - qa_core_node - 收到问题: 什么是ROS2
2025-09-22 15:30:45 - knowledge_base_server - 查询知识库: 什么是ROS2
2025-09-22 15:30:45 - qa_core_node - 发布答案: ROS2是机器人操作系统的下一代版本
```

## RViz可视化

可视化（另开终端）：
```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_qa_ws/install/setup.bash
ros2 run rviz2 rviz2
```
在 RViz 添加 Marker，订阅话题 /qa_visualization。

## 话题与服务
- 话题：/question, /answer, /web_question_input, /web_answer_output, /qa_visualization
- 服务：/query_knowledge_base（std_srvs/SetBool，request.data 放问题字符串）

## 故障排查
- rosbridge 未连通：
  - ros2 node list | grep rosbridge
  - ros2 launch rosbridge_server rosbridge_websocket_launch.xml
- 知识库 JSON 检查：
  - python3 -m json.tool src/ros2_qa_assistant/config/knowledge_base.json
  - 确保JSON格式正确，包含keywords数组和answer字段
  - 每个keyword对象必须包含word和weight字段
- 浏览器控制台检查 WebSocket 错误与 CORS 提示。

## 许可证
MIT