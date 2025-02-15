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

    // 구독 시작 메시지 전송 (비동기로 처리)
    setTimeout(() => {
      this.send('subscribe', { topic })
    }, 0)

    // 구독 해제 함수 반환
    return () => {
      this.unsubscribe(topic, callback)
    }
  }

  unsubscribe(topic, callback) {
    if (this.subscriptions.has(topic)) {
      this.subscriptions.get(topic).delete(callback)
      if (this.subscriptions.get(topic).size === 0) {
        this.subscriptions.delete(topic)
        // 구독 해제 메시지 전송
        this.send('unsubscribe', { topic })
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
    console.log('Raw message received:', message);  // 디버깅용
    
    try {
        // 메시지가 이미 파싱되었는지 확인
        const data = typeof message === 'string' ? JSON.parse(message) : message;
        const { type, topic, data: messageData } = data;

        console.log('Parsed message:', { type, topic, messageData });  // 디버깅용

        // type이 있고 해당 type이 구독된 경우
        if (type && this.subscriptions.has(type)) {
            this.subscriptions.get(type).forEach(callback => callback(data));
        }
        
        // topic 기반 구독 처리
        if (topic && this.subscriptions.has(topic)) {
            this.subscriptions.get(topic).forEach(callback => callback(messageData));
        }
    } catch (error) {
        console.error('Message handling error:', error);
        console.log('Failed to handle message:', message);
    }
  }

  send(type, data) {
    const message = JSON.stringify({ type, data })
    
    if (this.isConnected.value && this.ws.readyState === WebSocket.OPEN) {
      try {
        this.ws.send(message)
      } catch (error) {
        console.error('Send error:', error)
        this.pendingMessages.push(message)
      }
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