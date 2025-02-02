<template>
  <div class="camera-view">
    <div class="camera-header">
      <div class="camera-status-info">
        <div class="status-item">
          <span class="status-label">상태:</span>
          <span :class="['status-value', cameraStatus === '스트리밍 중' ? 'status-active' : '']">
            {{ cameraStatus }}
          </span>
        </div>
        <div v-if="streamInfo" class="status-item">
          <span class="status-label">해상도:</span>
          <span class="status-value">{{ streamInfo.width }}x{{ streamInfo.height }}</span>
        </div>
      </div>
    </div>
    
    <div class="camera-container">
      <div ref="fps" class="fps-display">FPS: 0.0</div>
      <video ref="videoElement" autoplay playsinline></video>
      <div class="debug-container" :class="{ 'debug-hidden': !showDebug }">
        <div class="debug-header">
          <button class="debug-toggle" @click="toggleDebug">
            {{ showDebug ? '로그 숨기기' : '로그 보기' }}
          </button>
        </div>
        <div ref="debug" class="debug-overlay" v-show="showDebug"></div>
      </div>
      <div v-if="currentSession" class="recording-status">
        녹화 중... (세션 ID: {{ currentSession }})
      </div>
    </div>

    <div class="camera-controls">
      <button 
        :class="['record-button', isRecording ? 'recording' : '']"
        @click="toggleRecording"
      >
        {{ isRecording ? '녹화 중지' : '녹화 시작' }}
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue';

const props = defineProps({
  robotId: {
    type: String,
    required: true
  },
  cameraType: {
    type: String,
    required: true,
    validator: (value) => ['front', 'rear'].includes(value)
  },
  rosHost: {
    type: String,
    default: '172.30.1.78'
  },
  rosPort: {
    type: Number,
    default: 9090
  }
});

const ws = ref(null);
const videoElement = ref(null);
const imageData = ref(null);
const cameraStatus = ref('연결 대기 중');
const errorMessage = ref('');
const hasPermission = ref(false);
const stream = ref(null);
const streamInfo = ref(null);
const video = ref(null);
const debug = ref(null);
const fps = ref(null);
const showDebug = ref(false);
const isRecording = ref(false);
const currentSession = ref(null);
let frameCount = 0;
let lastTime = performance.now();

function calculateFPS() {
  frameCount++;
  const now = performance.now();
  const elapsed = now - lastTime;
  
  if (elapsed >= 1000) {
    const currentFps = (frameCount * 1000) / elapsed;
    if (fps.value) {
      fps.value.textContent = `FPS: ${currentFps.toFixed(1)}`;
    }
    frameCount = 0;
    lastTime = now;
  }
  
  requestAnimationFrame(calculateFPS);
}

if (videoElement.value) {
  videoElement.value.addEventListener('play', () => {
    console.log('비디오 재생 시작');
    calculateFPS();
  });
}

function log(message) {
  console.log(message);
  if (debug.value) {
    debug.value.textContent += message + '\n';
  }
}

async function start() {
  try {
    log('WebRTC 연결 시작...');
    log(`카메라 연결 시도 - robotId: ${props.robotId}, type: ${props.cameraType}`);

    // 먼저 카메라 상태 확인
    const statusResponse = await fetch(`http://localhost:8000/api/v1/cameras/status/${props.robotId}_${props.cameraType}`, {
      credentials: 'include'
    });
    
    const statusResult = await statusResponse.json();
    if (!statusResult.success || statusResult.data.status === 'disconnected') {
      cameraStatus.value = '카메라 연결 안됨';
      log('카메라가 연결되어 있지 않습니다. WebRTC 연결을 시도하지 않습니다.');
      return;
    }

    const configuration = {
      iceServers: [
        {
          urls: [
            'stun:stun.l.google.com:19302',
            'stun:stun1.l.google.com:19302'
          ]
        }
      ],
      iceTransportPolicy: 'all'
    };
    
    log('PeerConnection 설정:', JSON.stringify(configuration));
    const pc = new RTCPeerConnection(configuration);
    
    // ICE 상태 모니터링 추가
    pc.oniceconnectionstatechange = () => {
      log('ICE 연결 상태:', pc.iceConnectionState);
      if (pc.iceConnectionState === 'failed') {
        log('ICE 연결 실패 - 재시도 필요');
        cameraStatus.value = 'ICE 연결 실패';
      } else if (pc.iceConnectionState === 'connected') {
        log('ICE 연결 성공');
        cameraStatus.value = '스트리밍 중';
      }
    };

    pc.onicegatheringstatechange = () => {
      log('ICE gathering 상태:', pc.iceGatheringState);
    };

    pc.onicecandidate = e => {
      if (e.candidate) {
        log('ICE candidate:', e.candidate.type);
      }
    };
    
    // 2. 이벤트 핸들러 설정
    pc.ontrack = function(event) {
      log('트랙 수신됨');
      log('트랙 종류: ' + event.track.kind);
      log('스트림 개수: ' + event.streams.length);
      if (event.track.kind === 'video') {
        log('비디오 트랙 설정 시도');
        if (videoElement.value) {
          try {
            videoElement.value.srcObject = event.streams[0];
            cameraStatus.value = '스트리밍 중';
            fetchCurrentSession();
            log('비디오 스트림 설정 완료');
          } catch (error) {
            log('비디오 스트림 설정 실패: ' + error.message);
          }
        } else {
          log('videoElement가 null입니다');
        }
      }
    };
    
    // 3. Offer 생성
    log('Offer 생성 중...');
    const offer = await pc.createOffer({
      offerToReceiveVideo: true
    });
    
    log('Local Description 설정 중...');
    await pc.setLocalDescription(offer);
    
    // 4. 서버에 offer 전송
    log('서버에 offer 전송 중...');
    const response = await fetch(`http://localhost:8000/api/v1/cameras/webrtc/${props.robotId}/${props.cameraType}/offer`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      credentials: 'include',
      body: JSON.stringify({
        sdp: pc.localDescription.sdp,
        type: pc.localDescription.type
      })
    });
    
    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(`서버 에러: ${response.status} - ${errorText}`);
    }
    
    const result = await response.json();
    if (!result.success) {
      throw new Error(result.message || '카메라 연결 실패');
    }

    const answer = result.data;
    log('서버로부터 응답 수신');
    log(`응답 type: ${answer.type}`);

    if (answer.type === 'disconnected') {
      cameraStatus.value = '연결 끊김';
      throw new Error('카메라가 연결되지 않았습니다.');
    }
    
    // 6. Remote Description 설정
    log('Remote Description 설정 중...');
    await pc.setRemoteDescription(new RTCSessionDescription({
      sdp: answer.sdp,
      type: answer.type
    }));
    log('WebRTC 연결 설정 완료');

  } catch (e) {
    log('에러 발생:');
    log(e.toString());
    log('에러 상세: ' + JSON.stringify(e));
    console.error(e);
    cameraStatus.value = '연결 실패';
  }
}

start();

const toggleDebug = () => {
  showDebug.value = !showDebug.value;
};

async function toggleRecording() {
  try {
    const action = isRecording.value ? 'stop' : 'start';
    const response = await fetch('http://localhost:8000/recording', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ action })
    });
    
    const result = await response.json();
    
    if (action === 'start') {
      isRecording.value = true;
      currentSession.value = result.session_id;
    } else {
      isRecording.value = false;
      currentSession.value = null;
    }
    
    log(`녹화 ${action}: ${result.session_id}`);
  } catch (error) {
    log('녹화 제어 에러: ' + error.message);
  }
}

// 현재 세션 ID를 가져오는 함수 추가
async function fetchCurrentSession() {
    try {
        const response = await fetch('http://localhost:8000/recording/current-session');
        const result = await response.json();
        if (result.session_id) {
            currentSession.value = result.session_id;
        }
    } catch (error) {
        log('세션 ID 조회 에러: ' + error.message);
    }
}

onMounted(() => {
  if (videoElement.value) {
    videoElement.value.addEventListener('play', () => {
      log('비디오 재생 시작');
      calculateFPS();
    });
    
    videoElement.value.addEventListener('error', (e) => {
      log('비디오 에러 발생: ' + e.target.error.message);
    });
    
    videoElement.value.addEventListener('loadedmetadata', () => {
      log('비디오 메타데이터 로드됨');
      log(`비디오 크기: ${videoElement.value.videoWidth}x${videoElement.value.videoHeight}`);
    });
  }
  start();
});
</script>

<style scoped>
.camera-view {
  width: 100%;
  max-width: 1200px; /* 최대 너비 설정 */
  margin: 0 auto;
  padding: 1rem;
  background: #f5f5f5;
  border-radius: 8px;
}

.camera-header {
  margin-bottom: 1rem;
}

.camera-container {
  position: relative;
  width: 100%;
  height: 0;
  padding-bottom: 75%; /* 4:3 비율 유지 (480/640 = 0.75) */
  background: #000;
  border-radius: 4px;
  overflow: hidden;
}

.camera-container video {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  object-fit: contain; /* 비디오 비율 유지 */
}

.fps-display {
  position: absolute;
  top: 10px;
  left: 10px;
  background: rgba(0, 0, 0, 0.7);
  color: white;
  padding: 5px 10px;
  border-radius: 5px;
  font-family: monospace;
  z-index: 2;
}

.debug-container {
  position: absolute;
  bottom: 10px;
  left: 10px;
  right: 10px;
  z-index: 2;
}

.debug-header {
  display: flex;
  justify-content: flex-end;
  margin-bottom: 5px;
}

.debug-toggle {
  background: rgba(0, 0, 0, 0.7);
  color: white;
  border: none;
  padding: 5px 10px;
  border-radius: 5px;
  cursor: pointer;
  font-size: 12px;
  transition: background-color 0.3s;
}

.debug-toggle:hover {
  background: rgba(0, 0, 0, 0.8);
}

.debug-overlay {
  background: rgba(0, 0, 0, 0.7);
  color: white;
  padding: 10px;
  font-size: 12px;
  font-family: monospace;
  max-height: 150px;
  overflow-y: auto;
  border-radius: 5px;
  transition: opacity 0.3s;
}

.debug-hidden .debug-overlay {
  display: none;
}

.camera-status-info {
  display: flex;
  gap: 1rem;
  font-size: 0.9rem;
  color: white;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.status-label {
  color: rgba(255, 255, 255, 0.7);
}

.status-value {
  font-weight: 500;
  color: white;
}

.status-active {
  color: #4CAF50;
}

.permission-request, .error-display {
  text-align: center;
  color: #fff;
  padding: 2rem;
}

.permission-button, .retry-button {
  margin-top: 1rem;
  padding: 0.5rem 1rem;
  background: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: background 0.3s;
}

.permission-button:hover, .retry-button:hover {
  background: #0056b3;
}

.loading {
  color: #fff;
  font-size: 1.1rem;
}

.camera-controls {
  margin-top: 1rem;
  display: flex;
  gap: 1rem;
  align-items: center;
}

.record-button {
  padding: 0.5rem 1rem;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  background: #dc3545;
  color: white;
  font-weight: 500;
}

.record-button.recording {
  background: #28a745;
  animation: pulse 2s infinite;
}

.recording-status {
  position: absolute;
  bottom: 10px;
  left: 10px;
  background: rgba(220, 53, 69, 0.8);
  color: white;
  padding: 5px 10px;
  border-radius: 5px;
  font-size: 0.9rem;
  z-index: 2;
}

@keyframes pulse {
  0% { opacity: 1; }
  50% { opacity: 0.5; }
  100% { opacity: 1; }
}
</style> 