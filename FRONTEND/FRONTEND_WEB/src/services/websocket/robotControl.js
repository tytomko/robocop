export class RobotControl {
    constructor(robotId, options = {}) {
        this.ws = null;
        this.robotId = robotId;
        this.options = options;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectTimeout = 1000; // 1초
    }

    connect() {
        try {
            // FastAPI 서버의 WebSocket 엔드포인트에 연결
            this.ws = new WebSocket(`ws://localhost:8000/ws/control/${this.robotId}`);
            
            this.ws.onopen = () => {
                console.log('제어 연결 성공');
                this.reconnectAttempts = 0;
                this.setupKeyboardControls();
                this.options.onConnect?.();
            };
            
            this.ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                if (data.status === 'error') {
                    console.error('제어 에러:', data.message);
                    this.options.onError?.(data.message);
                } else {
                    this.options.onFeedback?.(data);
                }
            };
            
            this.ws.onclose = () => {
                console.log('제어 연결 종료');
                this.cleanup();
                this.options.onDisconnect?.();
                this.tryReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket 에러:', error);
                this.options.onError?.('WebSocket 연결 에러');
            };
        } catch (error) {
            console.error('연결 시도 중 에러:', error);
            this.options.onError?.('연결 시도 중 에러 발생');
        }
    }

    tryReconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`재연결 시도 ${this.reconnectAttempts}/${this.maxReconnectAttempts}`);
            setTimeout(() => this.connect(), this.reconnectTimeout);
        }
    }

    setupKeyboardControls() {
        document.addEventListener('keydown', this.handleKeyDown);
        document.addEventListener('keyup', this.handleKeyUp);
    }

    handleKeyDown = (event) => {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
        
        const validKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
        if (validKeys.includes(event.key)) {
            event.preventDefault();
            console.log(`[Frontend] Sending keydown event: ${event.key}`);
            this.ws.send(JSON.stringify({
                key: event.key,
                action: 'keydown'
            }));
        }
    };

    handleKeyUp = (event) => {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
        
        const validKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
        if (validKeys.includes(event.key)) {
            event.preventDefault();
            console.log(`[Frontend] Sending keyup event: ${event.key}`);
            this.ws.send(JSON.stringify({
                key: event.key,
                action: 'keyup'
            }));
        }
    };

    cleanup() {
        document.removeEventListener('keydown', this.handleKeyDown);
        document.removeEventListener('keyup', this.handleKeyUp);
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        this.cleanup();
    }

    isConnected() {
        return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
    }
}

// 사용 예시:
/*
const robotControl = new RobotControl('robot1', {
    onConnect: () => {
        console.log('로봇 제어 연결됨');
    },
    onDisconnect: () => {
        console.log('로봇 제어 연결 해제됨');
    },
    onError: (error) => {
        console.error('에러 발생:', error);
    },
    onFeedback: (data) => {
        console.log('로봇 피드백:', data);
    }
});

// 연결 시작
robotControl.connect();

// 연결 해제
// robotControl.disconnect();
*/ 