<template>
  <div class="manual-control">
    <div class="header">
      <h2>수동 제어</h2>
      <div class="robot-selector">
        <select v-model="selectedRobot">
          <option value="">로봇 선택</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }} ({{ robot.id }})
          </option>
        </select>
      </div>
    </div>

    <div v-if="selectedRobot" class="control-panel">
      <div class="camera-view">
        <div class="camera-feed">
          <!-- 실제 카메라 피드가 들어갈 자리 -->
          <div class="placeholder">카메라 피드</div>
        </div>
        <div class="camera-controls">
          <button class="control-btn" @click="toggleCamera">
            {{ isCameraOn ? '카메라 끄기' : '카메라 켜기' }}
          </button>
          <button class="control-btn" @click="toggleLight">
            {{ isLightOn ? '조명 끄기' : '조명 켜기' }}
          </button>
        </div>
      </div>

      <div class="movement-controls">
        <div class="control-group">
          <h3>이동 제어</h3>
          <div class="direction-pad">
            <button class="direction-btn up" @click="move('forward')" :disabled="!isControlEnabled">
              ▲
            </button>
            <div class="middle-row">
              <button class="direction-btn left" @click="move('left')" :disabled="!isControlEnabled">
                ◀
              </button>
              <button class="direction-btn stop" @click="stop" :disabled="!isControlEnabled">
                ■
              </button>
              <button class="direction-btn right" @click="move('right')" :disabled="!isControlEnabled">
                ▶
              </button>
            </div>
            <button class="direction-btn down" @click="move('backward')" :disabled="!isControlEnabled">
              ▼
            </button>
          </div>
        </div>

        <div class="control-group">
          <h3>속도 제어</h3>
          <div class="speed-control">
            <input 
              type="range" 
              v-model="speed" 
              min="0" 
              max="100" 
              step="10"
              :disabled="!isControlEnabled"
            >
            <div class="speed-value">{{ speed }}%</div>
          </div>
        </div>
      </div>

      <div class="status-panel">
        <div class="status-item">
          <span class="label">현재 상태</span>
          <span class="value">{{ currentStatus }}</span>
        </div>
        <div class="status-item">
          <span class="label">배터리</span>
          <span class="value" :class="{ warning: batteryLevel < 20 }">
            {{ batteryLevel }}%
          </span>
        </div>
        <div class="status-item">
          <span class="label">연결 상태</span>
          <span class="value" :class="{ connected: isConnected }">
            {{ isConnected ? '연결됨' : '연결 끊김' }}
          </span>
        </div>
      </div>
    </div>

    <div v-else class="no-robot-selected">
      로봇을 선택해주세요
    </div>
  </div>
</template>

<script setup>
import { ref, computed } from 'vue'

const selectedRobot = ref('')
const isCameraOn = ref(false)
const isLightOn = ref(false)
const speed = ref(50)
const isConnected = ref(true)
const batteryLevel = ref(85)
const currentStatus = ref('대기 중')

const robots = ref([
  { id: 'SSAFY-ROBO-1', name: 'Robot 1' },
  { id: 'SSAFY-ROBO-2', name: 'Robot 2' }
])

const isControlEnabled = computed(() => {
  return selectedRobot.value && isConnected.value
})

const move = (direction) => {
  currentStatus.value = direction + ' 이동 중'
  // 실제 로봇 제어 명령 전송
  console.log(direction + ' 방향으로 이동, 속도: ' + speed.value + '%')
}

const stop = () => {
  currentStatus.value = '정지'
  // 실제 로봇 정지 명령 전송
  console.log('정지')
}

const toggleCamera = () => {
  isCameraOn.value = !isCameraOn.value
}

const toggleLight = () => {
  isLightOn.value = !isLightOn.value
}
</script>

<style scoped>
.manual-control {
  padding: 1.5rem;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 2rem;
}

.robot-selector select {
  padding: 0.5rem;
  border-radius: 4px;
  border: 1px solid #ddd;
  width: 200px;
}

.control-panel {
  display: grid;
  gap: 2rem;
}

.camera-view {
  background-color: white;
  border-radius: 8px;
  padding: 1.5rem;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.camera-feed {
  background-color: #000;
  aspect-ratio: 16/9;
  border-radius: 4px;
  margin-bottom: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.placeholder {
  color: #666;
}

.camera-controls {
  display: flex;
  gap: 1rem;
}

.movement-controls {
  display: grid;
  gap: 2rem;
}

.control-group {
  background-color: white;
  border-radius: 8px;
  padding: 1.5rem;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.control-group h3 {
  margin-bottom: 1rem;
  color: #333;
}

.direction-pad {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.middle-row {
  display: flex;
  gap: 0.5rem;
}

.direction-btn {
  width: 60px;
  height: 60px;
  border: none;
  border-radius: 8px;
  background-color: #007bff;
  color: white;
  font-size: 1.5rem;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
}

.direction-btn:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

.direction-btn.stop {
  background-color: #dc3545;
}

.speed-control {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.speed-control input[type="range"] {
  width: 100%;
}

.speed-value {
  font-weight: bold;
  color: #333;
}

.status-panel {
  background-color: white;
  border-radius: 8px;
  padding: 1.5rem;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1rem;
}

.status-item {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.label {
  color: #666;
  font-size: 0.875rem;
}

.value {
  font-weight: bold;
  color: #333;
}

.value.warning {
  color: #dc3545;
}

.value.connected {
  color: #28a745;
}

.no-robot-selected {
  text-align: center;
  padding: 3rem;
  background-color: white;
  border-radius: 8px;
  color: #666;
}

.control-btn {
  padding: 0.5rem 1rem;
  border: none;
  border-radius: 4px;
  background-color: #6c757d;
  color: white;
  cursor: pointer;
}

@media (max-width: 768px) {
  .status-panel {
    grid-template-columns: 1fr;
  }
  
  .direction-btn {
    width: 50px;
    height: 50px;
  }
}
</style> 