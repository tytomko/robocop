<template>
  <div class="robot-control">
    <!-- 메인 카메라/지도 영역 -->
    <div class="main-view" :class="{ 'map-view': isMapMainView }">
      <div class="camera-container" :class="{ secondary: isMapMainView }">
        <div class="camera-header">
          <h3>카메라 뷰</h3>
        </div>
        <div class="camera-content">
          <CameraView />
        </div>
      </div>
      <div class="map-container" :class="{ secondary: !isMapMainView }">
        <h3>위치 지도</h3>
        <MapView 
          :map-data="mapData" 
          :robot-position="robotPosition"
        />
        <button class="swap-view-btn" @click="toggleMainView">
          <i class="fas fa-exchange-alt"></i>
          화면 전환
        </button>
      </div>
    </div>

    <!-- 하단 제어 패널 -->
    <div class="control-panel">
      <!-- 로봇 상태 정보 -->
      <div class="status-panel">
        <h3>로봇 상태</h3>
        <div class="status-info">
          <div class="info-item">
            <span class="label">로봇명:</span>
            <span class="value">{{ selectedRobot?.name || '선택된 로봇 없음' }}</span>
          </div>
          <div class="info-item">
            <span class="label">상태:</span>
            <span class="value" :class="selectedRobot?.status">
              {{ getStatusText(selectedRobot?.status) }}
            </span>
          </div>
          <div class="info-item">
            <span class="label">배터리:</span>
            <div class="battery-indicator">
              <div 
                class="battery-level" 
                :style="{ 
                  width: (selectedRobot?.battery || 0) + '%',
                  backgroundColor: getBatteryColor(selectedRobot?.battery || 0)
                }"
              ></div>
              <span class="battery-text">{{ selectedRobot?.battery || 0 }}%</span>
            </div>
          </div>
        </div>
        <div class="control-buttons">
          <button class="control-btn stop" @click="stopControl">
            제어 종료
          </button>
          <button class="control-btn emergency" @click="emergencyStop">
            비상 정지
          </button>
        </div>
      </div>

      <!-- 수동 제어 패널 -->
      <div class="manual-control">
        <h3>수동 제어</h3>
        <div class="control-grid">
          <div class="direction-pad">
            <button class="direction-btn up" @click="move('forward')" :disabled="!isControlEnabled">
              <i class="fas fa-chevron-up"></i>
            </button>
            <button class="direction-btn left" @click="move('left')" :disabled="!isControlEnabled">
              <i class="fas fa-chevron-left"></i>
            </button>
            <button class="direction-btn stop" @click="move('stop')" :disabled="!isControlEnabled">
              <i class="fas fa-stop"></i>
            </button>
            <button class="direction-btn right" @click="move('right')" :disabled="!isControlEnabled">
              <i class="fas fa-chevron-right"></i>
            </button>
            <button class="direction-btn down" @click="move('backward')" :disabled="!isControlEnabled">
              <i class="fas fa-chevron-down"></i>
            </button>
          </div>
          <div class="speed-control">
            <label>이동 속도</label>
            <input 
              type="range" 
              v-model="speed" 
              min="1" 
              max="100" 
              :disabled="!isControlEnabled"
            >
            <span class="speed-value">{{ speed }}%</span>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import CameraView from '@/components/dashboard/monitoring/CameraView.vue'
import MapView from '@/components/map/MapView.vue'
import { webSocketService } from '@/services/websocket'
import { mapService } from '@/services/mapService'

// 상태 관리
const isMapMainView = ref(false)
const speed = ref(50)
const loading = ref(true)
const error = ref(null)

// 지도 데이터
const mapData = ref(null)
const robotPosition = ref({ x: 0, y: 0, theta: 0 })

// 로봇 상태 관리
const selectedRobot = ref(null)
const loadRobotData = async () => {
  loading.value = true
  error.value = null
  try {
    // API 호출 로직으로 대체 예정
    await new Promise(resolve => setTimeout(resolve, 1000))
    selectedRobot.value = {
      name: 'Robot 1',
      status: 'active',
      battery: 85
    }
  } catch (err) {
    error.value = '로봇 데이터를 불러오는데 실패했습니다.'
    console.error('로봇 데이터 로드 에러:', err)
  } finally {
    loading.value = false
  }
}

// 제어 가능 상태 계산
const isControlEnabled = computed(() => {
  return selectedRobot.value?.status === 'active'
})

// 상태 텍스트 변환
const getStatusText = (status) => {
  if (!status) return '알 수 없음'
  const statusMap = {
    active: '활동중',
    charging: '충전중',
    idle: '대기중',
    emergency: '비상정지'
  }
  return statusMap[status] || status
}

// 배터리 색상 계산
const getBatteryColor = (level) => {
  if (!level && level !== 0) return '#ccc'
  if (level > 60) return '#4CAF50'
  if (level > 20) return '#FFC107'
  return '#F44336'
}

// 화면 전환
const toggleMainView = () => {
  isMapMainView.value = !isMapMainView.value
}

// WebSocket 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/ws')
    
    webSocketService.subscribe('robot/position', (data) => {
      if (!data) return
      robotPosition.value = {
        x: data.x || 0,
        y: data.y || 0,
        theta: data.theta || 0
      }
    })

    webSocketService.subscribe('robot/status', (data) => {
      if (!data || !selectedRobot.value) return
      selectedRobot.value = {
        ...selectedRobot.value,
        ...data
      }
    })
  } catch (err) {
    console.error('웹소켓 연결 실패:', err)
    error.value = '실시간 데이터 연결에 실패했습니다.'
  }
}

// 지도 데이터 로드
const loadMapData = async () => {
  try {
    const mapImageUrl = '/maps/floor1.png'
    const mapMetadataUrl = '/maps/floor1.json'
    mapData.value = await mapService.loadMap(mapImageUrl, mapMetadataUrl)
  } catch (err) {
    console.error('지도 데이터 로드 실패:', err)
    error.value = '지도 데이터를 불러오는데 실패했습니다.'
  }
}

// 로봇 제어 명령 전송
const move = (direction) => {
  webSocketService.send('robot_control', {
    command: direction,
    speed: speed.value
  })
}

const stopControl = () => {
  webSocketService.send('robot_control', {
    command: 'stop'
  })
}

const emergencyStop = () => {
  webSocketService.send('robot_control', {
    command: 'emergency_stop'
  })
}

// 컴포넌트 마운트/언마운트 처리
onMounted(async () => {
  await loadRobotData()
  await loadMapData()
  await setupWebSocket()
})

onUnmounted(() => {
  webSocketService.disconnect()
})
</script>

<style scoped>
.robot-control {
  display: flex;
  flex-direction: column;
  height: 100%;
  width: 100%;
  gap: 20px;
  padding: 20px;
}

.main-view {
  display: grid;
  grid-template-columns: 3fr 1fr;
  gap: 20px;
  min-height: 70vh;
  margin-bottom: 20px;
}

.main-view.map-view {
  grid-template-columns: 1fr 3fr;
}

.camera-container,
.map-container {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  overflow: hidden;
  position: relative;
}

.camera-container.secondary,
.map-container.secondary {
  grid-column: 1;
}

.camera-container {
  display: flex;
  flex-direction: column;
}

.camera-header {
  padding: 15px;
  width: 100%;
  background: #f8f9fa;
  border-bottom: 1px solid #eee;
}

.camera-header h3 {
  margin: 0;
}

.camera-content {
  flex: 1;
  min-height: 0;
  padding: 0;
  min-height: calc(70vh - 50px);
  display: flex;
}

.camera-content > * {
  width: 100%;
  height: 100%;
}

.map-container h3 {
  padding: 15px;
  margin: 0;
  background: #f8f9fa;
  border-bottom: 1px solid #eee;
}

.map-placeholder {
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #f8f9fa;
  color: #666;
}

.swap-view-btn {
  position: absolute;
  bottom: 20px;
  right: 20px;
  padding: 10px 20px;
  background: white;
  border: 1px solid #ddd;
  border-radius: 4px;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 8px;
  z-index: 10;
}

.control-panel {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 20px;
  height: calc(30vh - 40px);
  min-height: 200px;
}

.status-panel,
.manual-control {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  padding: 20px;
}

.status-info {
  margin: 15px 0;
}

.info-item {
  display: flex;
  align-items: center;
  margin-bottom: 10px;
}

.info-item .label {
  width: 80px;
  color: #666;
}

.battery-indicator {
  width: 100px;
  height: 20px;
  background: #eee;
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
  width: 100%;
  text-align: center;
  color: #333;
  font-size: 0.85em;
  line-height: 20px;
}

.control-buttons {
  display: flex;
  gap: 10px;
  margin-top: 20px;
}

.control-btn {
  flex: 1;
  padding: 10px;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-weight: 500;
}

.control-btn.stop {
  background: #f5f5f5;
  color: #333;
}

.control-btn.emergency {
  background: #F44336;
  color: white;
}

.direction-pad {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 10px;
  margin: 20px 0;
  width: fit-content;
}

.direction-btn {
  width: 50px;
  height: 50px;
  border: none;
  border-radius: 4px;
  background: #f5f5f5;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.2em;
}

.direction-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.direction-btn.up { grid-column: 2; }
.direction-btn.left { grid-column: 1; grid-row: 2; }
.direction-btn.stop { grid-column: 2; grid-row: 2; background: #ff9800; color: white; }
.direction-btn.right { grid-column: 3; grid-row: 2; }
.direction-btn.down { grid-column: 2; grid-row: 3; }

.speed-control {
  margin-top: 20px;
}

.speed-control label {
  display: block;
  margin-bottom: 10px;
  color: #666;
}

.speed-control input[type="range"] {
  width: 100%;
  margin: 10px 0;
}

.speed-value {
  color: #333;
  font-weight: 500;
}
</style> 