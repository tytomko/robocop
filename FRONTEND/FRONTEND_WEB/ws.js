const ws = new WebSocket('ws://192.168.100.104:3000');

// 연결 상태 표시 요소
const connectionStatus = document.getElementById('connection-status');
const messageLog = document.getElementById('message-log');

// WebSocket 이벤트 핸들러
ws.onopen = () => {
    console.log('WebSocket connected');
    connectionStatus.textContent = '연결됨';
    connectionStatus.style.color = 'green';
    addMessageToLog('WebSocket 연결 성공');
    setupKeyboardControl();
};

ws.onmessage = (event) => {
    console.log('Received data:', event.data);
    try {
        const data = JSON.parse(event.data);
        addMessageToLog('수신: ' + JSON.stringify(data, null, 2));
    } catch (e) {
        addMessageToLog('수신: ' + event.data);
    }
};

ws.onerror = (error) => {
    console.error('WebSocket error:', error);
    connectionStatus.textContent = '연결 오류';
    connectionStatus.style.color = 'red';
    addMessageToLog('에러 발생: ' + error.message);
};

ws.onclose = () => {
    console.log('WebSocket connection closed');
    connectionStatus.textContent = '연결 끊김';
    connectionStatus.style.color = 'red';
    addMessageToLog('WebSocket 연결 종료');
};

// 키보드 제어 설정
function setupKeyboardControl() {
    const validKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
    const pressedKeys = new Set();

    document.addEventListener('keydown', (event) => {
        if (validKeys.includes(event.key)) {
            event.preventDefault();
            pressedKeys.add(event.key);
            sendCommand(event.key, 'keydown');
        }
    });

    document.addEventListener('keyup', (event) => {
        if (validKeys.includes(event.key)) {
            event.preventDefault();
            pressedKeys.delete(event.key);
            sendCommand(event.key, 'keyup');
        }
    });
}

// 명령 전송
function sendCommand(key, action) {
    const command = {
        key: key,
        action: action
    };
    
    ws.send(JSON.stringify(command));
}

// 메시지 로그에 추가
function addMessageToLog(message) {
    const timestamp = new Date().toLocaleTimeString();
    const messageElement = document.createElement('div');
    messageElement.className = 'message';
    messageElement.innerHTML = `
        <span class="timestamp">[${timestamp}]</span>
        <pre>${message}</pre>
    `;
    messageLog.appendChild(messageElement);
    messageLog.scrollTop = messageLog.scrollHeight;
}