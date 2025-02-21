class WebSocketService {
    constructor() {
        this.socket = null;
        this.subscribers = new Map();
    }

    async connect(url) {
        return new Promise((resolve, reject) => {
            try {
                this.socket = new WebSocket(url);
                
                this.socket.onopen = () => {
                    console.log('WebSocket 연결됨');
                    resolve();
                };

                this.socket.onmessage = (event) => {
                    try {
                        const data = JSON.parse(event.data);
                        console.log('받은 데이터:', data);  // 디버깅용
                        this.subscribers.forEach((callbacks, topic) => {
                            callbacks.forEach(callback => callback(data));
                        });
                    } catch (error) {
                        console.error('메시지 처리 중 에러:', error);
                    }
                };

                this.socket.onerror = (error) => {
                    console.error('WebSocket 에러:', error);
                    reject(error);
                };

                this.socket.onclose = () => {
                    console.log('WebSocket 연결 종료');
                    this.subscribers.clear();
                };

            } catch (error) {
                reject(error);
            }
        });
    }

    subscribe(topic, callback) {
        console.log('구독 시도:', topic);  // 디버깅용
        if (!this.socket) {
            throw new Error('웹소켓이 연결되지 않았습니다.');
        }
        
        if (!this.subscribers.has(topic)) {
            this.subscribers.set(topic, new Set());
        }
        this.subscribers.get(topic).add(callback);
        
        return () => {
            const callbacks = this.subscribers.get(topic);
            if (callbacks) {
                callbacks.delete(callback);
                if (callbacks.size === 0) {
                    this.subscribers.delete(topic);
                }
            }
        };
    }

    disconnect() {
        if (this.socket) {
            this.socket.close();
            this.socket = null;
            this.subscribers.clear();
        }
    }
}

// 싱글톤 인스턴스 생성 및 내보내기
const webSocketService = new WebSocketService();
export default webSocketService; 