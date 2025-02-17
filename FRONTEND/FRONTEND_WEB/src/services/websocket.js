import { ref } from 'vue'

class WebSocketService {
  constructor() {
    this.ws = null
    this.isConnected = ref(false)
    this.reconnectAttempts = 0
    this.maxReconnectAttempts = 5
    this.reconnectTimeout = 3000
    this.messageHandlers = new Map()
    this.subscriptions = new Map()
    this.pendingMessages = []
  }

  connect(url) {
    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(url)

        this.ws.onopen = () => {
          console.log('WebSocket 연결됨')
          this.isConnected.value = true
          this.reconnectAttempts = 0
          this.processPendingMessages()
          resolve()
        }

        this.ws.onclose = () => {
          console.log('WebSocket 연결 끊김')
          this.isConnected.value = false
          this.handleReconnect()
        }

        this.ws.onerror = (error) => {
          console.error('WebSocket 에러:', error)
          reject(error)
        }

        this.ws.onmessage = (event) => {
          try {
            const message = JSON.parse(event.data)
            this.handleMessage(message)
          } catch (error) {
            console.error('메시지 파싱 에러:', error)
          }
        }
      } catch (error) {
        reject(error)
      }
    })
  }

  handleReconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++
      console.log(`재연결 시도 ${this.reconnectAttempts}/${this.maxReconnectAttempts}...`)
      setTimeout(() => {
        this.connect(this.ws.url)
      }, this.reconnectTimeout)
    }
  }

  subscribe(topic, callback) {
    if (!this.subscriptions.has(topic)) {
      this.subscriptions.set(topic, new Set())
    }
    this.subscriptions.get(topic).add(callback)
  }

  unsubscribe(topic, callback) {
    if (this.subscriptions.has(topic)) {
      this.subscriptions.get(topic).delete(callback)
      if (this.subscriptions.get(topic).size === 0) {
        this.subscriptions.delete(topic)
      }
    }
  }

  registerHandler(type, handler) {
    if (!this.messageHandlers.has(type)) {
      this.messageHandlers.set(type, new Set())
    }
    this.messageHandlers.get(type).add(handler)
  }

  removeHandler(type, handler) {
    if (this.messageHandlers.has(type)) {
      this.messageHandlers.get(type).delete(handler)
    }
  }

  handleMessage(message) {
    try {
      const { type, topic, data } = message

      // ROS 토픽 메시지 처리
      if (type === 'ros_topic' && topic) {
        if (this.subscriptions.has(topic)) {
          this.subscriptions.get(topic).forEach(callback => callback(data))
        }
        
        // 일반 메시지 핸들러에도 전달
        if (this.messageHandlers.has(type)) {
          this.messageHandlers.get(type).forEach(handler => handler(message))
        }
        return
      }

      // 다른 타입의 메시지 처리
      if (type && this.messageHandlers.has(type)) {
        this.messageHandlers.get(type).forEach(handler => handler(data))
      }
    } catch (error) {
      console.error('Message handling error:', error)
    }
  }

  send(type, data) {
    const message = JSON.stringify({ type, data })
    
    if (this.isConnected.value && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(message)
    } else {
      this.pendingMessages.push(message)
    }
  }

  processPendingMessages() {
    while (this.pendingMessages.length > 0) {
      const message = this.pendingMessages.shift()
      this.ws.send(message)
    }
  }

  disconnect() {
    if (this.ws) {
      this.ws.close()
      this.subscriptions.clear()
      this.messageHandlers.clear()
      this.pendingMessages = []
    }
  }
}

export const webSocketService = new WebSocketService()

// 웹소켓 연결 및 메시지 처리
export const connectWebSocket = () => {
  const ws = new WebSocket('wss://robocopbackendssafy.duckdns.org/ws/test');

  ws.onopen = () => {
    console.log('WebSocket 연결됨');
  };

  ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    console.log('Raw message received:', message);

    // 메시지 타입에 따른 처리
    switch (message.type) {
      case 'connection_status':
        console.log('WebSocket connected');
        break;
      case 'ros_topic':  // 브릿지에서 받은 토픽 메시지
        handleRosTopic(message);
        break;
      default:
        console.log('Unknown message type:', message.type);
    }
  };

  ws.onerror = (error) => {
    console.error('WebSocket error:', error);
  };

  ws.onclose = () => {
    console.log('WebSocket 연결 종료');
    // 재연결 로직 추가
    setTimeout(() => {
      console.log('WebSocket 재연결 시도...');
      connectWebSocket();
    }, 3000);
  };

  return ws;
};

// ROS 토픽 메시지 처리
const handleRosTopic = (message) => {
  const { topic, data } = message;
  console.log(`Received ROS topic: ${topic}`);
  console.log('Topic data:', data);

  // 토픽별 처리 로직
  switch (topic) {
    case '/robot_1/status':
      // 로봇 상태 업데이트
      updateRobotStatus(data);
      break;
    // 다른 토픽들에 대한 처리 추가
    default:
      console.log('Unhandled topic:', topic);
  }
}; 