<template>
  <div class="realtime-monitoring">
    <div class="monitoring-header">
      <h3>실시간 모니터링</h3>
      <div class="monitoring-actions">
        <button class="refresh-btn" @click="refreshData">
          <span class="refresh-icon">⟳</span>
        </button>
      </div>
    </div>

    <draggable 
      v-model="monitoringComponents" 
      class="monitoring-sections"
      handle=".section-handle"
      item-key="id"
      @start="drag=true" 
      @end="drag=false"
    >
      <template #item="{element}">
        <div class="monitoring-section">
          <div class="section-header">
            <div class="section-handle">⋮⋮</div>
            <h4>{{ element.title }}</h4>
          </div>
          
          <!-- 로봇 상태 목록 -->
          <div v-if="element.type === 'robots'" class="robot-status-list">
            <div v-for="robot in robots" 
                :key="robot.id" 
                class="robot-status-card"
                :class="{ 'status-warning': robot.batteryLevel < 30, 
                          'status-danger': robot.batteryLevel < 15 }">
              <div class="robot-header">
                <span class="robot-name">{{ robot.name }}</span>
                <span class="status-badge" :class="robot.status">
                  {{ getStatusLabel(robot.status) }}
                </span>
              </div>

              <div class="status-details">
                <div class="status-item">
                  <span class="label">배터리</span>
                  <div class="battery-indicator">
                    <div class="battery-level" 
                        :style="{ width: robot.batteryLevel + '%',
                                backgroundColor: getBatteryColor(robot.batteryLevel) }">
                    </div>
                    <span class="battery-text">{{ robot.batteryLevel }}%</span>
                  </div>
                </div>

                <div class="status-item">
                  <span class="label">현재 위치</span>
                  <span class="value">{{ robot.location }}</span>
                </div>

                <div class="status-item">
                  <span class="label">현재 작업</span>
                  <span class="value">{{ robot.currentTask || '대기 중' }}</span>
                </div>

                <div class="status-item">
                  <span class="label">센서 상태</span>
                  <div class="sensor-status">
                    <span v-for="(status, sensor) in robot.sensors" 
                          :key="sensor"
                          class="sensor-badge"
                          :class="status">
                      {{ getSensorLabel(sensor) }}
                    </span>
                  </div>
                </div>
              </div>

              <div class="quick-actions">
                <button class="action-btn" 
                        @click="sendToCharge(robot)"
                        :disabled="robot.status === 'charging'">
                  충전소로 이동
                </button>
                <button class="action-btn" 
                        @click="emergencyStop(robot)"
                        :disabled="robot.status === 'stopped'">
                  긴급 정지
                </button>
              </div>
            </div>
          </div>

          <!-- 알림 목록 -->
          <div v-if="element.type === 'alerts'" class="alert-section">
            <div class="alert-list">
              <div v-for="alert in recentAlerts" 
                  :key="alert.id"
                  class="alert-item"
                  :class="alert.type">
                <span class="alert-time">{{ formatTime(alert.timestamp) }}</span>
                <p class="alert-message">{{ alert.message }}</p>
              </div>
            </div>
          </div>

          <!-- 로봇 지도 -->
          <MapView v-if="element.type === 'map'" class="robot-map-section" />
        </div>
      </template>
    </draggable>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import MapView from './MapView.vue'
import draggable from 'vuedraggable'
import { webSocketService } from '@/services/websocket'

// 모니터링 컴포넌트 순서 관리
const monitoringComponents = ref([
  { id: 1, type: 'robots', title: '로봇 목록' },
  { id: 2, type: 'alerts', title: '실시간 알림' },
  { id: 3, type: 'map', title: '로봇 지도' }
])

// 로봇 상태 데이터
const robots = ref([
  {
    id: 'SSAFY-ROBO-1',
    name: 'Robot 1',
    status: 'active',
    batteryLevel: 85,
    location: '1층 로비',
    currentTask: '순찰 중',
    sensors: {
      lidar: 'normal',
      camera: 'normal',
      battery: 'normal',
      motor: 'normal'
    }
  },
  {
    id: 'SSAFY-ROBO-2',
    name: 'Robot 2',
    status: 'charging',
    batteryLevel: 25,
    location: '충전소',
    currentTask: null,
    sensors: {
      lidar: 'normal',
      camera: 'warning',
      battery: 'warning',
      motor: 'normal'
    }
  }
])

// 최근 알림 데이터
const recentAlerts = ref([
  {
    id: 1,
    type: 'warning',
    timestamp: new Date(),
    message: 'Robot 2의 배터리가 부족합니다. (25%)'
  },
  {
    id: 2,
    type: 'info',
    timestamp: new Date(Date.now() - 5 * 60000),
    message: 'Robot 1이 순찰을 시작했습니다.'
  }
])

// 상태 레이블
const getStatusLabel = (status) => {
  const labels = {
    active: '활성',
    charging: '충전 중',
    stopped: '정지',
    error: '오류'
  }
  return labels[status] || status
}

// 센서 레이블
const getSensorLabel = (sensor) => {
  const labels = {
    lidar: 'LiDAR',
    camera: '카메라',
    battery: '배터리',
    motor: '모터'
  }
  return labels[sensor] || sensor
}

// 배터리 색상
const getBatteryColor = (level) => {
  if (level < 15) return '#dc3545'
  if (level < 30) return '#ffc107'
  return '#28a745'
}

// 시간 포맷
const formatTime = (date) => {
  return date.toLocaleTimeString('ko-KR', {
    hour: '2-digit',
    minute: '2-digit'
  })
}

// 데이터 갱신
const refreshData = () => {
  // API 호출 및 데이터 갱신 로직
  console.log('데이터 갱신')
}

// 로봇 제어 함수
const sendToCharge = (robot) => {
  console.log(robot.name + '를 충전소로 이동합니다.')
}

const emergencyStop = (robot) => {
  if (confirm(robot.name + '를 긴급 정지하시겠습니까?')) {
    console.log(robot.name + ' 긴급 정지')
  }
}

// WebSocket 연결 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/api/v1/robots/ws')
    
    // 로봇 상태 구독
    webSocketService.subscribe('monitoring/robots', (data) => {
      console.log('로봇 상태 업데이트:', data)
      updateRobotStatus(data)
    }, '/api/v1/robots/ws')

    // 알림 구독
    webSocketService.subscribe('monitoring/alerts', (data) => {
      addAlert(data)
    }, '/api/v1/robots/ws')

  } catch (error) {
    console.error('WebSocket 연결 실패:', error)
  }
}

// 로봇 상태 업데이트
const updateRobotStatus = (data) => {
  console.log('받은 데이터:', data)
  
  // 백엔드 데이터 형식에 맞게 매핑
  const robotData = {
    id: data.id,
    status: data.status,
    batteryLevel: data.battery,
    location: `${Math.round(data.location.x)}, ${Math.round(data.location.y)}`,
    currentTask: data.current_task || '대기 중',
    lastUpdated: data.last_updated
  }

  const robotIndex = robots.value.findIndex(r => r.id === robotData.id)
  if (robotIndex !== -1) {
    // 기존 데이터와 새로운 데이터 병합
    robots.value[robotIndex] = {
      ...robots.value[robotIndex],
      ...robotData
    }

    // 배터리 부족 알림 추가
    if (robotData.batteryLevel < 30) {
      addAlert({
        type: 'warning',
        message: `${robots.value[robotIndex].name}의 배터리가 부족합니다. (${robotData.batteryLevel}%)`
      })
    }
  } else {
    console.log('로봇을 찾을 수 없음:', robotData.id)
  }
}

// 알림 추가
const addAlert = (alert) => {
  recentAlerts.value.unshift({
    id: Date.now(),
    timestamp: new Date(),
    ...alert
  })

  // 최근 10개만 유지
  if (recentAlerts.value.length > 10) {
    recentAlerts.value.pop()
  }
}

// 컴포넌트 마운트/언마운트 시 WebSocket 연결/해제
onMounted(() => {
  setupWebSocket()
})

onUnmounted(() => {
  // isConnected 메서드 대신 connection 상태 확인
  if (webSocketService.connection) {
    webSocketService.disconnect()
  }
})
</script>

<style scoped>
.realtime-monitoring {
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 20px;
  padding: 20px;
}

.monitoring-sections {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 20px;
  overflow-y: auto;
}

.monitoring-section {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.section-header {
  display: flex;
  align-items: center;
  padding: 15px;
  border-bottom: 1px solid #eee;
}

.section-handle {
  cursor: move;
  padding: 0 10px;
  color: #666;
  font-size: 18px;
}

.section-header h4 {
  margin: 0;
  font-size: 16px;
  color: #333;
}

.robot-status-list {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
  padding: 20px;
}

.robot-status-card {
  background: white;
  border-radius: 8px;
  padding: 20px;
  border: 1px solid #eee;
}

.robot-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1rem;
}

.robot-name {
  font-weight: 500;
}

.status-badge {
  padding: 0.25rem 0.5rem;
  border-radius: 999px;
  font-size: 0.75rem;
  font-weight: 500;
}

.status-badge.active {
  background-color: #28a745;
  color: white;
}

.status-badge.charging {
  background-color: #007bff;
  color: white;
}

.status-badge.stopped {
  background-color: #dc3545;
  color: white;
}

.status-badge.error {
  background-color: #dc3545;
  color: white;
}

.status-details {
  display: flex;
  flex-direction: column;
  gap: 15px;
  margin: 15px 0;
}

.status-item {
  margin-bottom: 0.5rem;
}

.status-item .label {
  display: block;
  color: #666;
  font-size: 0.875rem;
  margin-bottom: 0.25rem;
}

.battery-indicator {
  width: 100%;
  height: 20px;
  background: #f0f0f0;
  border-radius: 10px;
  overflow: hidden;
  position: relative;
}

.battery-level {
  height: 100%;
  transition: width 0.3s ease;
}

.battery-text {
  position: absolute;
  right: 10px;
  top: 50%;
  transform: translateY(-50%);
  color: white;
  font-size: 12px;
  text-shadow: 0 0 2px rgba(0, 0, 0, 0.5);
}

.sensor-status {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
}

.sensor-badge {
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
}

.sensor-badge.normal {
  background: #28a745;
  color: white;
}

.sensor-badge.warning {
  background: #ffc107;
  color: #000;
}

.sensor-badge.error {
  background: #dc3545;
  color: white;
}

.quick-actions {
  display: flex;
  gap: 10px;
  margin-top: 15px;
}

.action-btn {
  flex: 1;
  padding: 8px;
  border: none;
  border-radius: 4px;
  background: #007bff;
  color: white;
  cursor: pointer;
  transition: background-color 0.3s;
}

.action-btn:hover:not(:disabled) {
  background: #0056b3;
}

.action-btn:disabled {
  background: #ccc;
  cursor: not-allowed;
}

.alert-section {
  padding: 20px;
}

.alert-list {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.alert-item {
  padding: 12px;
  border-radius: 4px;
  display: flex;
  flex-direction: column;
  gap: 5px;
}

.alert-item.warning {
  background: #fff3cd;
  border: 1px solid #ffeeba;
  color: #856404;
}

.alert-item.info {
  background: #cce5ff;
  border: 1px solid #b8daff;
  color: #004085;
}

.robot-map-section {
  height: 400px;
  padding: 20px;
}
</style>

<!-- 
실시간 모니터링 구현 방법 및 고려사항:

1. WebSocket 연결 관리
- 서버와의 실시간 양방향 통신을 위해 WebSocket 사용
- 연결 끊김 시 자동 재연결 로직 구현
- 연결 상태 모니터링 및 에러 처리

2. 데이터 업데이트 전략
- 실시간 데이터 수신 시 상태 즉시 업데이트
- 불필요한 리렌더링 방지를 위한 최적화
- 데이터 캐싱 및 임시 저장 구현

3. 성능 최적화
- 대량의 실시간 데이터 처리를 위한 가상 스크롤링
- 메모리 누수 방지를 위한 이벤트 리스너 정리
- 컴포넌트 언마운트 시 연결 정리

4. 오프라인 지원
- 네트워크 연결 끊김 시 대응 방안
- 오프라인 데이터 저장 및 동기화
- 사용자에게 연결 상태 표시

5. 보안
- WebSocket 연결 보안 (WSS)
- 메시지 암호화
- 인증 토큰 관리

6. 에러 처리
- 네트워크 오류 처리
- 잘못된 데이터 형식 처리
- 사용자 피드백 제공

7. 실시간 알림
- 중요 이벤트 발생 시 알림 표시
- 알림 우선순위 관리
- 알림 히스토리 관리

8. 모바일 대응
- 터치 이벤트 지원
- 반응형 레이아웃
- 배터리 효율성 고려

9. 확장성
- 새로운 데이터 타입 추가 용이성
- 다중 로봇 지원
- 커스텀 이벤트 처리
-->