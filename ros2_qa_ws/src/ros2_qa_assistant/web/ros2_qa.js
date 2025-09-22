(function(){
  let ros = null;
  let webQuestionTopic = null;
  let webAnswerTopic = null;
  let connected = false;
  let connecting = false;
  const pendingQueue = [];

  function byId(id){ return document.getElementById(id); }

  function connect(){
    const url = byId('ws-url').value.trim();
    if (!url) return;
    if (window.ROSLIB && window.ROSLIB.__stub) {
      byId('status').textContent = '库未加载：请联网或替换 web/roslib.min.js 为官方版本';
      return;
    }
  if (connecting || connected) return;
  connecting = true;
  ros = new window.ROSLIB.Ros({ url });
    byId('status').textContent = '连接中';

    ros.on('connection', function(){
      connecting = false;
      connected = true;
      byId('status').textContent = '已连接';
      byId('send-btn').disabled = false;
      console.log('[ros2-qa] WebSocket connected successfully');
      // 显式通过协议向rosbridge声明QoS，确保ROS2侧使用兼容QoS创建发布者/订阅者
      try {
        const qosProfile = { durability: 'transient_local', reliability: 'reliable', history: 'keep_last', depth: 10 };
        // 为 /web_question_input 创建带QoS的发布者（rosbridge在ROS2侧作为publisher）
        if (ros && typeof ros.callOnConnection === 'function') {
          ros.callOnConnection({
            op: 'advertise',
            topic: '/web_question_input',
            type: 'std_msgs/String',
            qos: qosProfile
          });
          console.log('[ros2-qa] advertised /web_question_input with QoS', qosProfile);
          // 为 /web_answer_output 创建带QoS的订阅者（rosbridge在ROS2侧作为subscriber）
          ros.callOnConnection({
            op: 'subscribe',
            topic: '/web_answer_output',
            type: 'std_msgs/String',
            qos: qosProfile
          });
          console.log('[ros2-qa] subscribed /web_answer_output with QoS', qosProfile);
        }
      } catch (e) {
        console.warn('[ros2-qa] explicit QoS advertise/subscribe failed:', e);
      }
      
      // 使用兼容rosbridge的QoS配置（若roslibjs支持，会一并带上；已通过上面callOnConnection强制声明）
      const qosOptions = { qos: { durability: 'transient_local', reliability: 'reliable', history: 'keep_last', depth: 10 } };
      
      webQuestionTopic = new window.ROSLIB.Topic({ 
        ros, 
        name: '/web_question_input', 
        messageType: 'std_msgs/String',
        ...qosOptions
      });
      try { if (typeof webQuestionTopic.advertise === 'function') webQuestionTopic.advertise(); } catch (e) { console.warn('[ros2-qa] advertise failed:', e); }
      console.log('[ros2-qa] webQuestionTopic created with QoS: transient_local, reliable');
      
  webAnswerTopic = new window.ROSLIB.Topic({ 
    ros, 
    name: '/web_answer_output', 
    messageType: 'std_msgs/String',
    qos: {
      durability: 'transient_local',
      reliability: 'reliable'
    }
  });
  webAnswerTopic.subscribe(function(msg){ 
    console.log('[ros2-qa] received answer:', msg.data);
    byId('answer-box').textContent = msg.data || ''; 
  });
  console.log('[ros2-qa] webAnswerTopic subscribed with QoS: transient_local, reliable');
  
  // Fallback subscribe directly to /answer in case the bridge node is not running
  const directAnswerTopic = new window.ROSLIB.Topic({ ros, name: '/answer', messageType: 'std_msgs/String' });
  directAnswerTopic.subscribe(function(msg){ 
    console.log('[ros2-qa] received direct answer:', msg.data);
    byId('answer-box').textContent = msg.data || ''; 
  });
      console.log('[ros2-qa] connected to', url);
      // Flush any pending messages
      while (pendingQueue.length > 0 && webQuestionTopic) {
        const text = pendingQueue.shift();
        try { webQuestionTopic.publish(new window.ROSLIB.Message({ data: text })); } catch (e) { console.error('flush publish failed', e); }
      }
    });
    ros.on('error', function(error){
      connecting = false; connected = false;
      byId('send-btn').disabled = true;
      console.error('[ros2-qa] WebSocket error:', error);
      byId('status').textContent = '连接错误：检查 rosbridge 是否运行，端口是否开放（默认 9090）';
    });
    ros.on('close', function(event){
      connecting = false; connected = false;
      byId('send-btn').disabled = true;
      console.log('[ros2-qa] WebSocket closed:', event);
      byId('status').textContent = '已断开：请确认 rosbridge 正在运行，并确保 WebSocket 地址正确';
    });
  }

  function send(){
    const q = byId('question').value.trim();
    if (!q) return;
    if (!connected || !webQuestionTopic) {
      // queue and try connect automatically
      if (pendingQueue.length < 5) pendingQueue.push(q);
      byId('status').textContent = '未连接，正在尝试连接后发送...';
      connect();
      return;
    }
    const msg = new window.ROSLIB.Message({ data: q });
    try {
      webQuestionTopic.publish(msg);
  console.log('[ros2-qa] published question:', q);
  byId('status').textContent = '已发送，等待回答...';
      // Fallback: also publish directly to /question in case the bridge node is not running
      try {
        const directQTopic = new window.ROSLIB.Topic({ ros, name: '/question', messageType: 'std_msgs/String' });
        directQTopic.publish(new window.ROSLIB.Message({ data: q }));
      } catch (e) { /* ignore fallback errors */ }
    } catch (e) {
      console.error('[ros2-qa] publish failed', e);
      byId('status').textContent = '发送失败：请检查控制台错误信息';
    }
  }

  byId('connect-btn').addEventListener('click', connect);
  byId('send-btn').addEventListener('click', send);
  // allow Ctrl+Enter to send
  byId('question').addEventListener('keydown', function(e){
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') send();
  });
  // disable send by default until connected
  byId('send-btn').disabled = true;
})();
