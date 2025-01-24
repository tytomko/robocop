<template>
  <div class="point-cloud-container">
    <div class="page-header">
      <h2>라이다 정보</h2>
      <div class="header-actions">
        <select v-model="selectedRobot" class="robot-select">
          <option value="">로봇 선택</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
      </div>
    </div>

    <div class="viewer-container">
      <canvas ref="canvas" class="point-cloud-canvas"></canvas>
    <div class="controls">
        <button @click="resetView">초기화</button>
        <div class="zoom-controls">
          <button @click="zoomIn">+</button>
          <button @click="zoomOut">-</button>
        </div>
      </div>
    </div>

    <div v-if="loading" class="loading-overlay">
      <div class="loading-spinner"></div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { webSocketService } from '@/services/websocket'

const canvas = ref(null)
const selectedRobot = ref('')
const loading = ref(false)
let ctx = null
let scale = 1
let offsetX = 0
let offsetY = 0
let isDragging = false
let lastX = 0
let lastY = 0

// 임시 로봇 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' }
])

// 캔버스 초기화
const initCanvas = () => {
  if (!canvas.value) {
    console.warn('캔버스 요소를 찾을 수 없습니다.')
    return
  }

  const container = canvas.value.parentElement
  if (!container) {
    console.warn('캔버스 컨테이너를 찾을 수 없습니다.')
    return
  }

  canvas.value.width = container.clientWidth
  canvas.value.height = container.clientHeight

  ctx = canvas.value.getContext('2d')
  if (!ctx) {
    console.error('2D 컨텍스트를 가져올 수 없습니다.')
    return
  }

  // 마우스 이벤트 리스너 설정
  canvas.value.addEventListener('mousedown', startDrag)
  canvas.value.addEventListener('mousemove', drag)
  canvas.value.addEventListener('mouseup', endDrag)
  canvas.value.addEventListener('wheel', handleWheel)
}

// 드래그 시작
const startDrag = (e) => {
  isDragging = true
  lastX = e.clientX
  lastY = e.clientY
}

// 드래그 중
const drag = (e) => {
  if (!isDragging) return

  const deltaX = e.clientX - lastX
  const deltaY = e.clientY - lastY
  
  offsetX += deltaX
  offsetY += deltaY
  
  lastX = e.clientX
  lastY = e.clientY
  
  drawPointCloud()
}

// 드래그 종료
const endDrag = () => {
  isDragging = false
}

// 휠 이벤트 처리
const handleWheel = (e) => {
  e.preventDefault()
  const delta = e.deltaY > 0 ? 0.9 : 1.1
  scale *= delta
  drawPointCloud()
}

// 확대
const zoomIn = () => {
  scale *= 1.1
  drawPointCloud()
}

// 축소
const zoomOut = () => {
  scale *= 0.9
  drawPointCloud()
}

// 뷰 초기화
const resetView = () => {
  scale = 1
  offsetX = 0
  offsetY = 0
  drawPointCloud()
}

// 포인트 클라우드 그리기
const drawPointCloud = (points) => {
  if (!ctx || !canvas.value) return

  ctx.clearRect(0, 0, canvas.value.width, canvas.value.height)
  
  // 그리드 그리기
  drawGrid()
  
  if (!points) return

  const centerX = canvas.value.width / 2 + offsetX
  const centerY = canvas.value.height / 2 + offsetY

  ctx.fillStyle = 'rgba(33, 150, 243, 0.6)'
  points.forEach(point => {
    const x = centerX + point.x * scale
    const y = centerY + point.y * scale
    
    ctx.beginPath()
    ctx.arc(x, y, 2, 0, Math.PI * 2)
    ctx.fill()
  })
}

// 그리드 그리기
const drawGrid = () => {
  if (!ctx || !canvas.value) return

  const width = canvas.value.width
  const height = canvas.value.height
  const gridSize = 50 * scale
  
  ctx.strokeStyle = '#ddd'
  ctx.lineWidth = 0.5
  
  // 수직선
  for (let x = offsetX % gridSize; x < width; x += gridSize) {
    ctx.beginPath()
    ctx.moveTo(x, 0)
    ctx.lineTo(x, height)
    ctx.stroke()
  }
  
  // 수평선
  for (let y = offsetY % gridSize; y < height; y += gridSize) {
    ctx.beginPath()
    ctx.moveTo(0, y)
    ctx.lineTo(width, y)
    ctx.stroke()
  }
}

// 웹소켓 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/ws')
    
    webSocketService.subscribe('lidar/points', (data) => {
      if (!data) return
      drawPointCloud(data.points)
    })
  } catch (error) {
    console.error('웹소켓 연결 실패:', error)
  }
}

// 컴포넌트 마운트
onMounted(async () => {
  initCanvas()
  await setupWebSocket()
  
  // 윈도우 리사이즈 이벤트 리스너
  window.addEventListener('resize', initCanvas)
})

// 컴포넌트 언마운트
onUnmounted(() => {
  if (canvas.value) {
    canvas.value.removeEventListener('mousedown', startDrag)
    canvas.value.removeEventListener('mousemove', drag)
    canvas.value.removeEventListener('mouseup', endDrag)
    canvas.value.removeEventListener('wheel', handleWheel)
  }
  window.removeEventListener('resize', initCanvas)
})
</script>

<style scoped>
.point-cloud-container {
  padding: 20px;
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
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

.viewer-container {
  flex: 1;
  position: relative;
  background: #1a1a1a;
  border-radius: 8px;
  overflow: hidden;
}

.point-cloud-canvas {
  width: 100%;
  height: 100%;
  cursor: move;
}

.controls {
  position: absolute;
  bottom: 20px;
  right: 20px;
  display: flex;
  gap: 10px;
}

.controls button {
  padding: 8px 16px;
  background: rgba(255, 255, 255, 0.9);
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 14px;
}

.controls button:hover {
  background: white;
}

.zoom-controls {
  display: flex;
  gap: 4px;
}

.zoom-controls button {
  padding: 8px 12px;
  font-weight: bold;
}

.loading-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.7);
  display: flex;
  justify-content: center;
  align-items: center;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 3px solid #f3f3f3;
  border-top: 3px solid #007bff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}
</style> 