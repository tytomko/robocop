class WebSocketService {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
    }

    connect(url) {
        return new Promise((resolve, reject) => {
            try {
                this.ws = new WebSocket(url);

                this.ws.onopen = () => {
                    console.log('WebSocket 연결됨');
                    this.isConnected = true;
                    this.reconnectAttempts = 0;
                    resolve();
                };

                this.ws.onclose = () => {
                    console.log('WebSocket 연결 끊김');
                    this.isConnected = false;
                    this.tryReconnect();
                };

                this.ws.onerror = (error) => {
                    console.error('WebSocket 에러:', error);
                    reject(error);
                };

            } catch (error) {
                console.error('WebSocket 연결 실패:', error);
                reject(error);
            }
        });
    }

    tryReconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`재연결 시도 ${this.reconnectAttempts}/${this.maxReconnectAttempts}`);
            setTimeout(() => this.connect(), 3000);
        }
    }

    send(data) {
        if (this.isConnected && this.ws) {
            this.ws.send(JSON.stringify(data));
        }
    }

    onMessage(callback) {
        if (this.ws) {
            this.ws.onmessage = (event) => {
                callback(event.data);
            };
        }
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
            this.isConnected = false;
        }
    }
}

export default new WebSocketService(); 