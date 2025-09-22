(function(){
  let ros = null;
  let webQuestionTopic = null;
  let webAnswerTopic = null;
  let connected = false;
  let connecting = false;
  const pendingQueue = [];
  let currentTypingMessage = null;

  // DOM元素
  const elements = {
    statusDot: () => document.getElementById('status-dot'),
    statusText: () => document.getElementById('status-text'),
    connectBtn: () => document.getElementById('connect-btn'),
    sendBtn: () => document.getElementById('send-btn'),
    wsUrl: () => document.getElementById('ws-url'),
    question: () => document.getElementById('question'),
    chatMessages: () => document.getElementById('chat-messages'),
    settingsPanel: () => document.getElementById('settings-panel'),
    settingsBtn: () => document.getElementById('settings-btn'),
    clearChatBtn: () => document.getElementById('clear-chat')
  };

  // 工具函数
  function scrollToBottom() {
    const chatMessages = elements.chatMessages();
    chatMessages.scrollTop = chatMessages.scrollHeight;
  }

  function updateConnectionStatus(status, text) {
    const statusDot = elements.statusDot();
    const statusText = elements.statusText();
    
    statusDot.className = `status-dot ${status}`;
    statusText.textContent = text;
  }

  function addMessage(content, type = 'ai', isTyping = false) {
    const chatMessages = elements.chatMessages();
    const messageDiv = document.createElement('div');
    messageDiv.className = `message-bubble ${type}-message`;
    
    if (isTyping) {
      messageDiv.innerHTML = `
        <div class="message-content typing-indicator">
          <span>AI正在思考</span>
          <div class="typing-dots">
            <span></span>
            <span></span>
            <span></span>
          </div>
        </div>
      `;
    } else {
      const messageContent = document.createElement('div');
      messageContent.className = 'message-content';
      messageContent.innerHTML = content;
      messageDiv.appendChild(messageContent);
    }
    
    chatMessages.appendChild(messageDiv);
    scrollToBottom();
    
    return messageDiv;
  }

  function typewriterEffect(element, text, speed = 50) {
    return new Promise((resolve) => {
      element.innerHTML = '';
      let i = 0;
      
      function typeChar() {
        if (i < text.length) {
          const char = text.charAt(i);
          const span = document.createElement('span');
          span.textContent = char;
          span.className = 'typing-effect';
          element.appendChild(span);
          i++;
          setTimeout(typeChar, speed);
        } else {
          // 移除打字效果类
          setTimeout(() => {
            element.innerHTML = text;
            resolve();
          }, 500);
        }
      }
      
      typeChar();
    });
  }

  function clearChat() {
    const chatMessages = elements.chatMessages();
    chatMessages.innerHTML = `
      <div class="welcome-message">
        <div class="message-bubble ai-message">
          <div class="message-content">
            <p>对话已清空！有什么问题可以继续问我。</p>
          </div>
        </div>
      </div>
    `;
  }

  function toggleSettings() {
    const settingsPanel = elements.settingsPanel();
    settingsPanel.classList.toggle('show');
  }

  function autoResizeTextarea() {
    const textarea = elements.question();
    textarea.style.height = 'auto';
    textarea.style.height = Math.min(textarea.scrollHeight, 120) + 'px';
  }

  function connect() {
    const url = elements.wsUrl().value.trim();
    if (!url) return;
    
    if (window.ROSLIB && window.ROSLIB.__stub) {
      updateConnectionStatus('error', '库未加载：请联网或替换roslib.min.js');
      return;
    }

    if (connecting || connected) return;
    
    connecting = true;
    updateConnectionStatus('connecting', '连接中...');
    elements.connectBtn().disabled = true;

    ros = new window.ROSLIB.Ros({ url });

    ros.on('connection', function(){
      connecting = false;
      connected = true;
      updateConnectionStatus('connected', '已连接');
      elements.sendBtn().disabled = false;
      elements.connectBtn().disabled = false;
      elements.connectBtn().innerHTML = '<i class="fas fa-check"></i><span>已连接</span>';
      
      console.log('[ros2-qa] WebSocket connected successfully');
      
      // 隐藏设置面板
      elements.settingsPanel().classList.remove('show');
      
      // 添加连接成功消息
      addMessage('<p>成功连接到ROS系统！现在可以开始提问了。</p>');

      // QoS配置
      try {
        const qosProfile = { 
          durability: 'transient_local', 
          reliability: 'reliable', 
          history: 'keep_last', 
          depth: 10 
        };

        if (ros && typeof ros.callOnConnection === 'function') {
          ros.callOnConnection({
            op: 'advertise',
            topic: '/web_question_input',
            type: 'std_msgs/String',
            qos: qosProfile
          });
          
          // 注意：不在这里订阅，避免与后面的webAnswerTopic.subscribe重复
        }
      } catch (e) {
        console.warn('[ros2-qa] QoS setup failed:', e);
      }

      // 创建话题
      const qosOptions = { 
        qos: { 
          durability: 'transient_local', 
          reliability: 'reliable', 
          history: 'keep_last', 
          depth: 10 
        } 
      };

      webQuestionTopic = new window.ROSLIB.Topic({ 
        ros, 
        name: '/web_question_input', 
        messageType: 'std_msgs/String',
        ...qosOptions
      });

      webAnswerTopic = new window.ROSLIB.Topic({ 
        ros, 
        name: '/web_answer_output', 
        messageType: 'std_msgs/String',
        ...qosOptions
      });

      webAnswerTopic.subscribe(function(msg){ 
        const timestamp = new Date().toISOString();
        console.log(`[ros2-qa] ${timestamp} received answer:`, msg.data);
        console.log('[ros2-qa] Call stack:', new Error().stack);
        
        // 移除打字指示器
        if (currentTypingMessage) {
          currentTypingMessage.remove();
          currentTypingMessage = null;
        }

        const answer = msg.data || '抱歉，我现在无法回答这个问题。';
        const messageElement = addMessage('');
        const contentElement = messageElement.querySelector('.message-content');
        
        // 使用打字机效果显示答案
        typewriterEffect(contentElement, answer, 30);
      });

      // 发送待处理的消息
      while (pendingQueue.length > 0 && webQuestionTopic) {
        const text = pendingQueue.shift();
        try { 
          webQuestionTopic.publish(new window.ROSLIB.Message({ data: text }));
          currentTypingMessage = addMessage('', 'ai', true);
        } catch (e) { 
          console.error('flush publish failed', e); 
        }
      }
    });

    ros.on('error', function(error){
      connecting = false; 
      connected = false;
      updateConnectionStatus('error', '连接失败');
      elements.sendBtn().disabled = true;
      elements.connectBtn().disabled = false;
      elements.connectBtn().innerHTML = '<i class="fas fa-plug"></i><span>重新连接</span>';
      
      console.error('[ros2-qa] WebSocket error:', error);
      addMessage('<p>连接失败！请检查：</p><ul><li>rosbridge是否运行</li><li>端口9090是否开放</li><li>网络连接是否正常</li></ul>', 'ai');
    });

    ros.on('close', function(){
      connecting = false; 
      connected = false;
      updateConnectionStatus('error', '连接断开');
      elements.sendBtn().disabled = true;
      elements.connectBtn().disabled = false;
      elements.connectBtn().innerHTML = '<i class="fas fa-plug"></i><span>重新连接</span>';
      
      console.log('[ros2-qa] WebSocket closed');
      addMessage('<p>与ROS系统的连接已断开。</p>', 'ai');
    });
  }

  function sendQuestion() {
    const questionText = elements.question().value.trim();
    if (!questionText) return;

    // 添加用户消息
    addMessage(questionText, 'user');
    
    // 清空输入框
    elements.question().value = '';
    autoResizeTextarea();

    if (!connected || !webQuestionTopic) {
      pendingQueue.push(questionText);
      addMessage('<p>消息已加入队列，等待连接建立后发送...</p>', 'ai');
      return;
    }

    try {
      webQuestionTopic.publish(new window.ROSLIB.Message({ data: questionText }));
      console.log('[ros2-qa] sent question:', questionText);
      
      // 添加打字指示器
      currentTypingMessage = addMessage('', 'ai', true);
      
      // 10秒后如果没有回复，显示超时消息
      setTimeout(() => {
        if (currentTypingMessage && document.body.contains(currentTypingMessage)) {
          currentTypingMessage.remove();
          currentTypingMessage = null;
          addMessage('<p>响应超时，请检查ROS系统状态或重试。</p>', 'ai');
        }
      }, 10000);
      
    } catch (e) {
      console.error('[ros2-qa] send failed:', e);
      addMessage('<p>发送失败，请重试。</p>', 'ai');
    }
  }

  // 事件监听器
  document.addEventListener('DOMContentLoaded', function() {
    // 连接按钮
    elements.connectBtn().addEventListener('click', connect);
    
    // 发送按钮
    elements.sendBtn().addEventListener('click', sendQuestion);
    
    // 设置按钮
    elements.settingsBtn().addEventListener('click', toggleSettings);
    
    // 清除对话按钮
    elements.clearChatBtn().addEventListener('click', clearChat);
    
    // 输入框事件
    const questionInput = elements.question();
    questionInput.addEventListener('input', autoResizeTextarea);
    questionInput.addEventListener('keydown', function(e) {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        sendQuestion();
      }
    });

    // WebSocket URL输入框回车连接
    elements.wsUrl().addEventListener('keydown', function(e) {
      if (e.key === 'Enter') {
        connect();
      }
    });
  });

})();