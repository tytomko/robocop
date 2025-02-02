<template>
  <div class="map-view">
    <div class="map-header">
      <h3>맵 뷰</h3>
      <div class="map-controls">
        <button class="control-btn" @click="zoomIn">
          <span>+</span>
        </button>
        <button class="control-btn" @click="zoomOut">
          <span>-</span>
        </button>
        <button class="control-btn" @click="resetView">
          <span>↺</span>
        </button>
      </div>
    </div>

    <div class="map-container">
      <div class="robot-selector">
        <select v-model="selectedRobot" class="robot-select" @change="handleRobotSelection">
          <option value="">로봇 선택</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
      </div>
      
      <canvas 
        ref="canvasRef"
        class="map-canvas"
        @mousedown="startDrag"
        @mousemove="drag"
        @mouseup="endDrag"
        @mouseleave="endDrag"
        @wheel="handleWheel"
      ></canvas>

      <div v-if="loading" class="loading-overlay">
        <div class="spinner"></div>
        <span>맵 데이터를 불러오는 중...</span>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue'
import { webSocketService } from '@/services/websocket'

const canvasRef = ref(null)
const selectedRobot = ref(localStorage.getItem('selectedRobot') || '')
const loading = ref(true)
const isDragging = ref(false)
const lastPos = ref({ x: 0, y: 0 })
const offset = ref({ x: 0, y: 0 })
const scale = ref(1)

// 임시 로봇 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' },
  { id: 'robot3', name: 'Robot 3' }
])

// 로봇 선택 처리
const handleRobotSelection = () => {
  if (selectedRobot.value) {
    localStorage.setItem('selectedRobot', selectedRobot.value)
  } else {
    localStorage.removeItem('selectedRobot')
  }
  drawMap()
}

// 드래그 관련 함수들
const startDrag = (event) => {
  isDragging.value = true
  lastPos.value = {
    x: event.clientX,
    y: event.clientY
  }
}

const drag = (event) => {
  if (!isDragging.value) return

  const deltaX = event.clientX - lastPos.value.x
  const deltaY = event.clientY - lastPos.value.y
  
  offset.value = {
    x: offset.value.x + deltaX,
    y: offset.value.y + deltaY
  }

  lastPos.value = {
    x: event.clientX,
    y: event.clientY
  }

  drawMap()
}

const endDrag = () => {
  isDragging.value = false
}

// 줌 관련 함수들
const handleWheel = (event) => {
  event.preventDefault()
  const delta = event.deltaY * -0.01
  const newScale = Math.min(Math.max(0.1, scale.value + delta), 5)
  scale.value = newScale
  drawMap()
}

const zoomIn = () => {
  scale.value = Math.min(5, scale.value + 0.1)
  drawMap()
}

const zoomOut = () => {
  scale.value = Math.max(0.1, scale.value - 0.1)
  drawMap()
}

const resetView = () => {
  scale.value = 1
  offset.value = { x: 0, y: 0 }
  drawMap()
}

// 맵 그리기
const drawMap = () => {
  if (!canvasRef.value) return

  const canvas = canvasRef.value
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  // 캔버스 크기 설정
  const container = canvas.parentElement
  if (!container) return

  canvas.width = container.clientWidth
  canvas.height = container.clientHeight

  // 맵 데이터가 없으면 기본 그리드 표시
  drawDefaultGrid(ctx, canvas.width, canvas.height)

  // 선택된 로봇 위치 표시
  if (selectedRobot.value) {
    drawRobot(ctx, canvas.width/2, canvas.height/2)
  }
}

// 기본 그리드 그리기
const drawDefaultGrid = (ctx, width, height) => {
  ctx.save()
  ctx.translate(offset.value.x, offset.value.y)
  ctx.scale(scale.value, scale.value)

  ctx.clearRect(-width, -height, width * 3, height * 3)
  ctx.strokeStyle = '#ddd'
  ctx.lineWidth = 1

  const gridSize = 50
  const startX = Math.floor(-width/scale.value - offset.value.x)
  const startY = Math.floor(-height/scale.value - offset.value.y)
  const endX = Math.ceil(width/scale.value - offset.value.x)
  const endY = Math.ceil(height/scale.value - offset.value.y)

  // 수직선
  for (let x = startX; x <= endX; x += gridSize) {
    ctx.beginPath()
    ctx.moveTo(x, startY)
    ctx.lineTo(x, endY)
    ctx.stroke()
  }

  // 수평선
  for (let y = startY; y <= endY; y += gridSize) {
    ctx.beginPath()
    ctx.moveTo(startX, y)
    ctx.lineTo(endX, y)
    ctx.stroke()
  }

  ctx.restore()
}

// 로봇 그리기
const drawRobot = (ctx, x, y) => {
  ctx.save()
  ctx.translate(offset.value.x + x * scale.value, offset.value.y + y * scale.value)
  
  // 로봇 본체
  ctx.beginPath()
  ctx.arc(0, 0, 15 * scale.value, 0, Math.PI * 2)
  ctx.fillStyle = '#007bff'
  ctx.fill()
  
  // 방향 표시
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(20 * scale.value, 0)
  ctx.strokeStyle = 'white'
  ctx.lineWidth = 2 * scale.value
  ctx.stroke()
  
  ctx.restore()
}

// WebSocket 연결 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/api/v1/robots/ws')
    
    // 로봇 위치 구독
    webSocketService.subscribe('robot/position', (data) => {
      console.log('로봇 위치 업데이트:', data)
      drawMap()
    }, '/api/v1/robots/ws')

  } catch (error) {
    console.error('WebSocket 연결 실패:', error)
  }
}

// 컴포넌트 마운트/언마운트
onMounted(() => {
  setupWebSocket()
  window.addEventListener('resize', drawMap)
  loading.value = false
})

onUnmounted(() => {
  if (webSocketService.isConnected()) {
    webSocketService.disconnect()
  }
  window.removeEventListener('resize', drawMap)
})

// 선택된 로봇이 변경될 때마다 맵 다시 그리기
watch(selectedRobot, () => {
  drawMap()
})
</script>

<style scoped>
.map-view {
  height: 100%;
  display: flex;
  flex-direction: column;
  background: white;
}

.map-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20px;
  border-bottom: 1px solid #eee;
}

.map-header h3 {
  margin: 0;
  color: #333;
}

.map-controls {
  display: flex;
  gap: 10px;
}

.control-btn {
  width: 32px;
  height: 32px;
  border: 1px solid #ddd;
  border-radius: 4px;
  background: white;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 18px;
  color: #666;
  transition: all 0.3s;
}

.control-btn:hover {
  background: #f8f9fa;
  color: #333;
}

.map-container {
  flex: 1;
  position: relative;
  overflow: hidden;
}

.robot-selector {
  position: absolute;
  top: 20px;
  left: 20px;
  z-index: 10;
  background: white;
  border-radius: 4px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  padding: 10px;
}

.robot-select {
  width: 200px;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  background: white;
  font-size: 14px;
}

.map-canvas {
  width: 100%;
  height: 100%;
  cursor: grab;
}

.map-canvas:active {
  cursor: grabbing;
}

.loading-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(255, 255, 255, 0.9);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 20px;
}

.spinner {
  width: 40px;
  height: 40px;
  border: 4px solid #f3f3f3;
  border-top: 4px solid #007bff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}
</style> 