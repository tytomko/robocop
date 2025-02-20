<template>
  <div class="sensor-data">
    <div class="page-header">
      <h2>센서 데이터</h2>
      <div class="header-actions">
        <select v-model="selectedRobot" class="robot-select">
          <option value="">로봇 선택</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
        <button class="refresh-btn" @click="refreshData">
          <i class="fas fa-sync-alt"></i>
        </button>
      </div>
    </div>

    <div class="sensor-grid">
      <!-- LiDAR 데이터 -->
      <div class="sensor-card lidar-card">
        <div class="card-header">
          <h3>LiDAR 데이터</h3>
          <div class="card-actions">
            <button class="action-btn" @click="toggleLidarView">
              {{ isLidarExpanded ? '축소' : '확대' }}
            </button>
          </div>
        </div>
        <div class="card-content" :class="{ expanded: isLidarExpanded }">
          <canvas ref="lidarCanvas" class="lidar-canvas"></canvas>
        </div>
      </div>

      <!-- 모터 상태 -->
      <div class="sensor-card">
        <div class="card-header">
          <h3>모터 상태</h3>
        </div>
        <div class="card-content">
          <div class="motor-status">
            <div class="motor-group">
              <h4>왼쪽 모터</h4>
              <div class="status-item">
                <span class="label">속도:</span>
                <span class="value">{{ motorStatus.left.speed }} RPM</span>
              </div>
              <div class="status-item">
                <span class="label">전류:</span>
                <span class="value">{{ motorStatus.left.current }} A</span>
              </div>
              <div class="status-item">
                <span class="label">온도:</span>
                <span class="value">{{ motorStatus.left.temperature }}°C</span>
              </div>
            </div>
            <div class="motor-group">
              <h4>오른쪽 모터</h4>
              <div class="status-item">
                <span class="label">속도:</span>
                <span class="value">{{ motorStatus.right.speed }} RPM</span>
              </div>
              <div class="status-item">
                <span class="label">전류:</span>
                <span class="value">{{ motorStatus.right.current }} A</span>
              </div>
              <div class="status-item">
                <span class="label">온도:</span>
                <span class="value">{{ motorStatus.right.temperature }}°C</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- IMU 데이터 -->
      <div class="sensor-card">
        <div class="card-header">
          <h3>IMU 데이터</h3>
        </div>
        <div class="card-content">
          <div class="imu-data">
            <div class="data-group">
              <h4>자이로스코프</h4>
              <div class="data-item">
                <span class="label">X:</span>
                <span class="value">{{ imuData.gyro.x }} rad/s</span>
              </div>
              <div class="data-item">
                <span class="label">Y:</span>
                <span class="value">{{ imuData.gyro.y }} rad/s</span>
              </div>
              <div class="data-item">
                <span class="label">Z:</span>
                <span class="value">{{ imuData.gyro.z }} rad/s</span>
              </div>
            </div>
            <div class="data-group">
              <h4>가속도계</h4>
              <div class="data-item">
                <span class="label">X:</span>
                <span class="value">{{ imuData.accel.x }} m/s²</span>
              </div>
              <div class="data-item">
                <span class="label">Y:</span>
                <span class="value">{{ imuData.accel.y }} m/s²</span>
              </div>
              <div class="data-item">
                <span class="label">Z:</span>
                <span class="value">{{ imuData.accel.z }} m/s²</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- 배터리 상태 -->
      <div class="sensor-card">
        <div class="card-header">
          <h3>배터리 상태</h3>
        </div>
        <div class="card-content">
          <div class="battery-status">
            <div class="battery-level">
              <div class="level-bar" :style="{ width: batteryStatus.level + '%', backgroundColor: getBatteryColor(batteryStatus.level) }"></div>
              <span class="level-text">{{ batteryStatus.level }}%</span>
            </div>
            <div class="battery-details">
              <div class="detail-item">
                <span class="label">전압:</span>
                <span class="value">{{ batteryStatus.voltage }}V</span>
              </div>
              <div class="detail-item">
                <span class="label">전류:</span>
                <span class="value">{{ batteryStatus.current }}A</span>
              </div>
              <div class="detail-item">
                <span class="label">온도:</span>
                <span class="value">{{ batteryStatus.temperature }}°C</span>
              </div>
              <div class="detail-item">
                <span class="label">예상 사용 시간:</span>
                <span class="value">{{ batteryStatus.remainingTime }}분</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { webSocketService } from '@/services/websocket'

// 상태 관리
const selectedRobot = ref('')
const isLidarExpanded = ref(false)
const lidarCanvas = ref(null)
let lidarCtx = null

// 임시 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' }
])

const motorStatus = ref({
  left: { speed: 0, current: 0, temperature: 0 },
  right: { speed: 0, current: 0, temperature: 0 }
})

const imuData = ref({
  gyro: { x: 0, y: 0, z: 0 },
  accel: { x: 0, y: 0, z: 0 }
})

const batteryStatus = ref({
  level: 85,
  voltage: 24.5,
  current: 2.1,
  temperature: 35,
  remainingTime: 120
})

// LiDAR 캔버스 초기화
const initLidarCanvas = () => {
  if (!lidarCanvas.value) {
    console.warn('LiDAR 캔버스 요소를 찾을 수 없습니다.')
    return
  }

  const canvas = lidarCanvas.value
  const context = canvas.getContext('2d')
  if (!context) {
    console.error('2D 컨텍스트를 가져올 수 없습니다.')
    return
  }
  lidarCtx = context
  
  // 캔버스 크기 설정
  const container = canvas.parentElement
  if (!container) {
    console.warn('캔버스 컨테이너를 찾을 수 없습니다.')
    return
  }
  
  canvas.width = container.clientWidth || 400  // 기본값 설정
  canvas.height = container.clientHeight || 300  // 기본값 설정
}

// LiDAR 데이터 그리기
const drawLidarData = (data) => {
  if (!lidarCtx || !lidarCanvas.value || !data) {
    console.warn('LiDAR 데이터를 그리는데 필요한 요소가 없습니다.')
    return
  }

  const canvas = lidarCanvas.value
  const centerX = canvas.width / 2
  const centerY = canvas.height / 2
  const scale = Math.min(canvas.width, canvas.height) / 2 - 20

  // 캔버스 클리어
  lidarCtx.clearRect(0, 0, canvas.width, canvas.height)

  // 배경 그리드 그리기
  lidarCtx.strokeStyle = '#eee'
  lidarCtx.beginPath()
  for (let r = 1; r <= 5; r++) {
    lidarCtx.arc(centerX, centerY, scale * r / 5, 0, Math.PI * 2)
  }
  lidarCtx.stroke()

  // LiDAR 데이터 포인트 그리기
  lidarCtx.fillStyle = 'rgba(33, 150, 243, 0.5)'
  data.forEach(point => {
    if (!point || typeof point.distance !== 'number' || typeof point.angle !== 'number') {
      return  // 유효하지 않은 포인트는 건너뜀
    }
    
    const distance = point.distance * scale
    const angle = point.angle
    const x = centerX + distance * Math.cos(angle)
    const y = centerY + distance * Math.sin(angle)

    lidarCtx.beginPath()
    lidarCtx.arc(x, y, 2, 0, Math.PI * 2)
    lidarCtx.fill()
  })
}

// 웹소켓 연결 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/ws')
    
    // 모터 상태 구독
    webSocketService.subscribe('sensor/motor', (data) => {
      if (!data) return
      motorStatus.value = {
        left: {
          speed: data.left?.speed ?? 0,
          current: data.left?.current ?? 0,
          temperature: data.left?.temperature ?? 0
        },
        right: {
          speed: data.right?.speed ?? 0,
          current: data.right?.current ?? 0,
          temperature: data.right?.temperature ?? 0
        }
      }
    })

    // IMU 데이터 구독
    webSocketService.subscribe('sensor/imu', (data) => {
      if (!data) return
      imuData.value = {
        gyro: {
          x: data.gyro?.x ?? 0,
          y: data.gyro?.y ?? 0,
          z: data.gyro?.z ?? 0
        },
        accel: {
          x: data.accel?.x ?? 0,
          y: data.accel?.y ?? 0,
          z: data.accel?.z ?? 0
        }
      }
    })

    // 배터리 상태 구독
    webSocketService.subscribe('sensor/battery', (data) => {
      if (!data) return
      batteryStatus.value = {
        level: data.level ?? 0,
        voltage: data.voltage ?? 0,
        current: data.current ?? 0,
        temperature: data.temperature ?? 0,
        remainingTime: data.remainingTime ?? 0
      }
    })

    // LiDAR 데이터 구독
    webSocketService.subscribe('sensor/lidar', (data) => {
      drawLidarData(data)
    })

  } catch (error) {
    console.error('웹소켓 연결 실패:', error)
  }
}

// 배터리 색상 계산
const getBatteryColor = (level) => {
  if (level > 60) return '#4CAF50'
  if (level > 20) return '#FFC107'
  return '#F44336'
}

// LiDAR 뷰 토글
const toggleLidarView = () => {
  isLidarExpanded.value = !isLidarExpanded.value
  // 크기 변경 후 캔버스 재초기화
  setTimeout(() => {
    initLidarCanvas()
  }, 300)
}

// 데이터 새로고침
const refreshData = () => {
  if (selectedRobot.value) {
    webSocketService.send('request_sensor_data', {
      robotId: selectedRobot.value,
      types: ['motor', 'imu', 'battery', 'lidar']
    })
  }
}

// 라이프사이클 훅
onMounted(async () => {
  await setupWebSocket()
  initLidarCanvas()

  // 임시 LiDAR 데이터 생성 (테스트용)
  setInterval(() => {
    const testData = []
    for (let i = 0; i < 360; i += 2) {
      testData.push({
        angle: i * Math.PI / 180,
        distance: Math.random() * 0.8 + 0.2
      })
    }
    drawLidarData(testData)
  }, 100)
})

onUnmounted(() => {
  webSocketService.disconnect()
})
</script>

<style scoped>
.sensor-data {
  padding: 20px;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.header-actions {
  display: flex;
  gap: 10px;
}

.robot-select {
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  min-width: 150px;
}

.refresh-btn {
  padding: 8px;
  border: none;
  border-radius: 4px;
  background: #f5f5f5;
  cursor: pointer;
}

.refresh-btn:hover {
  background: #e9ecef;
}

.sensor-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
  flex: 1;
  overflow-y: auto;
}

.sensor-card {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  overflow: hidden;
}

.lidar-card {
  grid-column: span 2;
  grid-row: span 2;
}

.card-header {
  padding: 15px;
  background: #f8f9fa;
  border-bottom: 1px solid #eee;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.card-header h3 {
  margin: 0;
}

.card-content {
  padding: 15px;
  height: 300px;
  transition: height 0.3s ease;
}

.card-content.expanded {
  height: 600px;
}

.lidar-canvas {
  width: 100%;
  height: 100%;
}

.motor-status,
.imu-data {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 20px;
}

.motor-group,
.data-group {
  background: #f8f9fa;
  padding: 15px;
  border-radius: 4px;
}

.motor-group h4,
.data-group h4 {
  margin: 0 0 10px 0;
  color: #333;
}

.status-item,
.data-item {
  display: flex;
  justify-content: space-between;
  margin-bottom: 8px;
}

.label {
  color: #666;
}

.battery-status {
  padding: 15px;
}

.battery-level {
  height: 24px;
  background: #eee;
  border-radius: 12px;
  margin-bottom: 20px;
  position: relative;
  overflow: hidden;
}

.level-bar {
  height: 100%;
  transition: width 0.3s ease;
}

.level-text {
  position: absolute;
  width: 100%;
  text-align: center;
  line-height: 24px;
  color: #333;
}

.battery-details {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
}

.detail-item {
  display: flex;
  justify-content: space-between;
  background: #f8f9fa;
  padding: 8px;
  border-radius: 4px;
}

.action-btn {
  padding: 6px 12px;
  border: 1px solid #ddd;
  border-radius: 4px;
  background: white;
  cursor: pointer;
}

.action-btn:hover {
  background: #f5f5f5;
}
</style> 