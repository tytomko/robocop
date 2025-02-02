import { ref } from 'vue'

class WebSocketService {
  constructor() {
    this.connections = new Map() // 여러 웹소켓 연결을 관리하기 위한 맵
    this.isConnected = ref(false)
    this.reconnectAttempts = new Map()
    this.maxReconnectAttempts = 5
    this.reconnectTimeout = 3000
    this.messageHandlers = new Map()
    this.subscriptions = new Map()
    this.pendingMessages = new Map()
  }

  getConnectionKey(url) {
    // URL에서 경로를 추출하여 연결 키로 사용
    try {
      const urlObj = new URL(url)
      return urlObj.pathname
    } catch (e) {
      return url
    }
  }

  connect(url) {
    return new Promise((resolve, reject) => {
      try {
        const connectionKey = this.getConnectionKey(url)
        
        // 이미 연결이 존재하면 재사용
        if (this.connections.has(connectionKey) && this.connections.get(connectionKey).readyState === WebSocket.OPEN) {
          console.log(`기존 WebSocket 연결 재사용: ${connectionKey}`)
          resolve()
          return
        }

        const ws = new WebSocket(url)
        this.connections.set(connectionKey, ws)
        this.reconnectAttempts.set(connectionKey, 0)
        this.pendingMessages.set(connectionKey, [])

        ws.onopen = () => {
          console.log(`WebSocket 연결됨: ${connectionKey}`)
          this.isConnected.value = true
          this.reconnectAttempts.set(connectionKey, 0)
          this.processPendingMessages(connectionKey)
          resolve()
        }

        ws.onclose = () => {
          console.log(`WebSocket 연결 끊김: ${connectionKey}`)
          this.isConnected.value = false
          this.handleReconnect(connectionKey, url)
        }

        ws.onerror = (error) => {
          console.error(`WebSocket 에러 (${connectionKey}):`, error)
          reject(error)
        }

        ws.onmessage = (event) => {
          try {
            const message = JSON.parse(event.data)
            this.handleMessage(connectionKey, message)
          } catch (error) {
            console.error(`메시지 파싱 에러 (${connectionKey}):`, error)
          }
        }
      } catch (error) {
        reject(error)
      }
    })
  }

  handleReconnect(connectionKey, url) {
    const attempts = this.reconnectAttempts.get(connectionKey) || 0
    if (attempts < this.maxReconnectAttempts) {
      this.reconnectAttempts.set(connectionKey, attempts + 1)
      console.log(`재연결 시도 ${attempts + 1}/${this.maxReconnectAttempts} (${connectionKey})...`)
      setTimeout(() => {
        this.connect(url)
      }, this.reconnectTimeout)
    }
  }

  subscribe(topic, callback, connectionKey = '/ws') {
    if (!this.subscriptions.has(connectionKey)) {
      this.subscriptions.set(connectionKey, new Map())
    }
    
    const topicMap = this.subscriptions.get(connectionKey)
    if (!topicMap.has(topic)) {
      topicMap.set(topic, new Set())
    }
    
    topicMap.get(topic).add(callback)
    this.send('subscribe', { topic }, connectionKey)

    return () => {
      this.unsubscribe(topic, callback, connectionKey)
    }
  }

  unsubscribe(topic, callback, connectionKey = '/ws') {
    const topicMap = this.subscriptions.get(connectionKey)
    if (topicMap?.has(topic)) {
      topicMap.get(topic).delete(callback)
      if (topicMap.get(topic).size === 0) {
        topicMap.delete(topic)
        this.send('unsubscribe', { topic }, connectionKey)
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

  handleMessage(connectionKey, message) {
    const { type, topic, data } = message
    const topicMap = this.subscriptions.get(connectionKey)

    if (topic && topicMap?.has(topic)) {
      topicMap.get(topic).forEach(callback => callback(data))
    }

    if (type && this.messageHandlers.has(type)) {
      this.messageHandlers.get(type).forEach(handler => handler(data))
    }
  }

  send(type, data, connectionKey = '/ws') {
    const message = JSON.stringify({ type, data })
    const ws = this.connections.get(connectionKey)
    
    if (ws?.readyState === WebSocket.OPEN) {
      ws.send(message)
    } else {
      if (!this.pendingMessages.has(connectionKey)) {
        this.pendingMessages.set(connectionKey, [])
      }
      this.pendingMessages.get(connectionKey).push(message)
    }
  }

  processPendingMessages(connectionKey) {
    const messages = this.pendingMessages.get(connectionKey) || []
    const ws = this.connections.get(connectionKey)
    
    while (messages.length > 0 && ws?.readyState === WebSocket.OPEN) {
      const message = messages.shift()
      ws.send(message)
    }
  }

  disconnect(connectionKey) {
    if (connectionKey) {
      const ws = this.connections.get(connectionKey)
      if (ws) {
        ws.close()
        this.connections.delete(connectionKey)
        this.subscriptions.delete(connectionKey)
        this.pendingMessages.delete(connectionKey)
        this.reconnectAttempts.delete(connectionKey)
      }
    } else {
      // 모든 연결 종료
      for (const [key, ws] of this.connections) {
        ws.close()
      }
      this.connections.clear()
      this.subscriptions.clear()
      this.pendingMessages.clear()
      this.reconnectAttempts.clear()
    }
  }
}

export const webSocketService = new WebSocketService() 