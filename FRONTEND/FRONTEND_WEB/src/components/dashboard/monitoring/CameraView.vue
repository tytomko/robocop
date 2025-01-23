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
      <div v-if="!hasPermission" class="permission-request">
        <p>카메라 접근 권한이 필요합니다</p>
        <button @click="requestPermission" class="permission-button">
          카메라 권한 요청
        </button>
      </div>
      <div v-else-if="errorMessage" class="error-display">
        <p>{{ errorMessage }}</p>
        <button @click="retryConnection" class="retry-button">
          다시 시도
        </button>
      </div>
      <video v-else-if="hasPermission" ref="videoElement" autoplay playsinline></video>
      <div v-else class="loading">
        카메라 연결 중...
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue';

const ws = ref(null);
const videoElement = ref(null);
const imageData = ref(null);
const cameraStatus = ref('연결 대기 중');
const errorMessage = ref('');
const hasPermission = ref(false);
const stream = ref(null);
const streamInfo = ref(null);

const requestPermission = async () => {
  try {
    if (!navigator.mediaDevices) {
      throw new Error('이 브라우저는 카메라 접근을 지원하지 않습니다.');
    }

    // 기존 스트림이 있다면 정리
    if (stream.value) {
      stream.value.getTracks().forEach(track => {
        track.stop();
      });
    }

    // 카메라 스트림 요청
    stream.value = await navigator.mediaDevices.getUserMedia({ 
      video: {
        width: { ideal: 640 },
        height: { ideal: 480 },
        frameRate: { ideal: 30 }
      }
    });
    
    if (!stream.value) {
      throw new Error('카메라 스트림을 가져올 수 없습니다.');
    }

    // 비디오 엘리먼트에 스트림 연결
    if (videoElement.value) {
      videoElement.value.srcObject = stream.value;
      hasPermission.value = true;
      errorMessage.value = '';
    }

    // 스트림 정보 저장
    const videoTrack = stream.value.getVideoTracks()[0];
    if (videoTrack) {
      const settings = videoTrack.getSettings();
      streamInfo.value = {
        width: settings.width || 640,
        height: settings.height || 480
      };
    }

  } catch (error) {
    console.error('카메라 권한 에러:', error);
    hasPermission.value = false;
    errorMessage.value = error.name === 'NotAllowedError' 
      ? '카메라 접근이 거부되었습니다. 브라우저 설정에서 카메라 권한을 허용해주세요.'
      : '카메라 연결 중 오류가 발생했습니다.';
  }
};

const connectWebSocket = () => {
  if (ws.value && ws.value.readyState === WebSocket.OPEN) {
    console.log('이미 WebSocket이 연결되어 있습니다.');
    return;
  }

  try {
    const wsUrl = 'ws://localhost:8080';
    ws.value = new WebSocket(`${wsUrl}/ws/camera/ROBOT_001`);
    console.log('WebSocket 연결 시도:', `${wsUrl}/ws/camera/ROBOT_001`);
    
    ws.value.onopen = () => {
      console.log('카메라 WebSocket 연결됨');
      cameraStatus.value = '연결됨';
      errorMessage.value = '';
      
      // 연결 성공 시 카메라 스트림 시작 요청
      try {
        ws.value.send(JSON.stringify({ 
          action: 'start_stream',
          resolution: streamInfo.value ? {
            width: streamInfo.value.width,
            height: streamInfo.value.height
          } : null
        }));
      } catch (error) {
        console.error('스트림 시작 요청 실패:', error);
      }
    };

    ws.value.onmessage = (event) => {
      try {
        // 바이너리 데이터 처리
        if (event.data instanceof Blob) {
          const reader = new FileReader();
          reader.onload = () => {
            try {
              const base64data = reader.result.split(',')[1];
              imageData.value = base64data;
              cameraStatus.value = '스트리밍 중';
              errorMessage.value = ''; // 성공적인 프레임 수신 시 에러 메시지 초기화
            } catch (error) {
              console.error('프레임 데이터 처리 실패:', error);
            }
          };
          reader.onerror = (error) => {
            console.error('프레임 읽기 실패:', error);
          };
          reader.readAsDataURL(event.data);
          return;
        }

        // JSON 메시지 처리
        const data = JSON.parse(event.data);
        console.log('수신된 메시지:', data);
        
        if (data.error) {
          throw new Error(data.error);
        } else if (data.status) {
          cameraStatus.value = data.status;
          if (data.status === 'error') {
            errorMessage.value = data.message || '카메라 스트리밍 오류가 발생했습니다.';
          }
        }
      } catch (error) {
        console.error('메시지 처리 중 에러:', error);
        handleError(error);
      }
    };

    ws.value.onerror = (error) => {
      console.error('WebSocket 에러:', error);
      handleError(error);
    };

    ws.value.onclose = (event) => {
      console.log('WebSocket 연결 종료:', {
        code: event.code,
        reason: event.reason,
        wasClean: event.wasClean
      });
      
      // 정상적인 종료가 아닌 경우에만 재연결 시도
      if (!event.wasClean) {
        handleReconnect(event);
      } else {
        cameraStatus.value = '연결 종료';
      }
    };
  } catch (error) {
    console.error('WebSocket 연결 실패:', error);
    handleError(error);
  }
};

const retryConnection = () => {
  errorMessage.value = '';
  connectWebSocket();
};

// 에러 처리 함수
const handleError = (error) => {
  console.error('카메라 에러 발생:', error);
  errorMessage.value = error.message || '카메라 연결 중 오류가 발생했습니다.';
  cameraStatus.value = '에러';
};

// 재연결 처리 함수
const handleReconnect = (event) => {
  const maxRetries = 3;
  const retryDelay = 3000;
  let retryCount = 0;

  const tryReconnect = () => {
    if (retryCount < maxRetries) {
      retryCount++;
      console.log(`재연결 시도 ${retryCount}/${maxRetries}`);
      cameraStatus.value = `재연결 시도 중 (${retryCount}/${maxRetries})`;
      
      // 기존 연결 정리
      if (ws.value) {
        try {
          ws.value.close();
        } catch (error) {
          console.error('WebSocket 정리 중 에러:', error);
        }
      }
      
      setTimeout(() => {
        connectWebSocket();
      }, retryDelay);
    } else {
      cameraStatus.value = '연결 실패';
      errorMessage.value = '연결을 설정할 수 없습니다. 페이지를 새로고침하여 다시 시도해주세요.';
    }
  };

  tryReconnect();
};

onMounted(async () => {
  console.log('카메라 컴포넌트 마운트됨');
  try {
    await requestPermission();
  } catch (error) {
    console.error('카메라 초기화 실패:', error);
    handleError(error);
  }
});

onUnmounted(() => {
  console.log('카메라 컴포넌트 언마운트: 리소스 정리 시작');
  
  // WebSocket 연결 정리
  if (ws.value) {
    try {
      ws.value.close(1000, '사용자가 페이지를 떠났습니다');
      console.log('WebSocket 연결 정상 종료됨');
    } catch (error) {
      console.error('WebSocket 정리 중 에러:', error);
    }
  }
  
  // 카메라 스트림 정리
  if (stream.value) {
    try {
      stream.value.getTracks().forEach(track => {
        track.stop();
        console.log(`카메라 트랙 정리됨: ${track.kind}`);
      });
    } catch (error) {
      console.error('카메라 스트림 정리 중 에러:', error);
    }
  }
  
  console.log('카메라 컴포넌트 언마운트: 리소스 정리 완료');
});
</script>

<style scoped>
.camera-view {
  padding: 1rem;
  background: #f5f5f5;
  border-radius: 8px;
}

.camera-header {
  margin-bottom: 1rem;
}

.camera-status-info {
  display: flex;
  gap: 1rem;
  font-size: 0.9rem;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.status-label {
  color: #666;
}

.status-value {
  font-weight: 500;
}

.status-active {
  color: #28a745;
}

.camera-container {
  position: relative;
  width: 100%;
  aspect-ratio: 16/9;
  background: #000;
  display: flex;
  align-items: center;
  justify-content: center;
  overflow: hidden;
  border-radius: 4px;
}

.camera-container video {
  width: 100%;
  height: 100%;
  object-fit: contain;
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
</style> 