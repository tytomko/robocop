import { ref } from 'vue'

class WebSocketService {
  constructor() {
    this.ws = null
    this.handlers = new Map()
    this.isConnected = false
    this.reconnectAttempts = 0
    this.maxReconnectAttempts = 5
    this.reconnectTimeout = 3000
    this.messageHandlers = new Map()
    this.subscriptions = new Map()
    this.pendingMessages = []
  }

  // 싱글톤 인스턴스
  static getInstance() {
    if (!this.instance) {
      this.instance = new WebSocketService()
    }
    return this.instance
  }

  // 웹소켓 연결
  async connect(url) {
    if (this.ws) return
    
    this.ws = new WebSocket(url)
    
    this.ws.onmessage = (event) => {
      const message = JSON.parse(event.data)
      const handlers = this.handlers.get(message.type) || []
      handlers.forEach(handler => handler(message))
    }

    return new Promise((resolve, reject) => {
      this.ws.onopen = () => {
        this.isConnected = true
        this.reconnectAttempts = 0
        this.processPendingMessages()
        resolve()
      }
      this.ws.onclose = () => {
        console.log('WebSocket 연결 끊김')
        this.isConnected = false
        this.handleReconnect()
      }
      this.ws.onerror = reject
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
    if (!this.handlers.has(type)) {
      this.handlers.set(type, [])
    }
    this.handlers.get(type).push(handler)
  }

  removeHandler(type, handler) {
    if (this.handlers.has(type)) {
      const handlers = this.handlers.get(type)
      const index = handlers.indexOf(handler)
      if (index > -1) {
        handlers.splice(index, 1)
      }
    }
  }

  handleMessage(message) {
    const handlers = this.handlers.get(message.type) || []
    handlers.forEach(handler => {
      try {
        handler(message)
      } catch (error) {
        console.error('Handler error:', error)
      }
    })
  }

  send(type, data) {
    const message = JSON.stringify({ type, data })
    
    if (this.isConnected && this.ws.readyState === WebSocket.OPEN) {
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

export const webSocketService = WebSocketService.getInstance() 