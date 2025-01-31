<template>
  <div class="map-viewer" ref="mapContainer">
    <canvas ref="mapCanvas" @wheel="handleZoom" @mousedown="startPan" @mousemove="pan" @mouseup="stopPan" @mouseleave="stopPan"></canvas>
    <div class="map-controls">
      <button class="control-btn" @click="zoomIn">
        <i class="fas fa-plus"></i>
      </button>
      <button class="control-btn" @click="zoomOut">
        <i class="fas fa-minus"></i>
      </button>
      <button class="control-btn" @click="resetView">
        <i class="fas fa-home"></i>
      </button>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue'

const props = defineProps({
  mapData: {
    type: Object,
    required: true,
    default: () => ({
      width: 800,
      height: 600,
      resolution: 0.05, // meters per pixel
      origin: { x: 0, y: 0 },
      data: new Uint8Array(800 * 600) // 예시 데이터
    })
  },
  robotPosition: {
    type: Object,
    required: true,
    default: () => ({ x: 0, y: 0, theta: 0 })
  }
})

// Canvas 참조
const mapContainer = ref(null)
const mapCanvas = ref(null)
let ctx = null

// 뷰어 상태
const viewState = ref({
  scale: 1,
  offsetX: 0,
  offsetY: 0,
  isPanning: false,
  lastX: 0,
  lastY: 0
})

// 캔버스 초기화
const initCanvas = () => {
  const canvas = mapCanvas.value
  const container = mapContainer.value
  
  // 캔버스 크기 설정
  canvas.width = container.clientWidth
  canvas.height = container.clientHeight
  
  ctx = canvas.getContext('2d')
  ctx.imageSmoothingEnabled = false
}

// 지도 그리기
const drawMap = () => {
  if (!ctx) return

  // 캔버스 클리어
  ctx.clearRect(0, 0, mapCanvas.value.width, mapCanvas.value.height)
  
  // 변환 매트릭스 저장
  ctx.save()
  
  // 뷰 변환 적용
  ctx.translate(viewState.value.offsetX, viewState.value.offsetY)
  ctx.scale(viewState.value.scale, viewState.value.scale)

  // 지도 그리기
  const imageData = ctx.createImageData(props.mapData.width, props.mapData.height)
  for (let i = 0; i < props.mapData.data.length; i++) {
    const value = props.mapData.data[i]
    const idx = i * 4
    imageData.data[idx] = value     // R
    imageData.data[idx + 1] = value // G
    imageData.data[idx + 2] = value // B
    imageData.data[idx + 3] = 255   // A
  }
  ctx.putImageData(imageData, 0, 0)

  // 로봇 위치 그리기
  drawRobot()

  // 변환 매트릭스 복원
  ctx.restore()
}

// 로봇 그리기
const drawRobot = () => {
  const { x, y, theta } = props.robotPosition
  const robotSize = 20 // 픽셀 단위

  ctx.save()
  ctx.translate(x, y)
  ctx.rotate(theta)

  // 로봇 본체
  ctx.beginPath()
  ctx.arc(0, 0, robotSize / 2, 0, Math.PI * 2)
  ctx.fillStyle = '#2196F3'
  ctx.fill()

  // 방향 표시
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(robotSize / 2, 0)
  ctx.strokeStyle = 'white'
  ctx.lineWidth = 2
  ctx.stroke()

  ctx.restore()
}

// 줌 제어
const handleZoom = (e) => {
  e.preventDefault()
  const delta = e.deltaY
  const scaleChange = delta > 0 ? 0.9 : 1.1
  
  const rect = mapCanvas.value.getBoundingClientRect()
  const mouseX = e.clientX - rect.left
  const mouseY = e.clientY - rect.top

  const newScale = viewState.value.scale * scaleChange
  if (newScale >= 0.5 && newScale <= 5) {
    // 마우스 위치 기준으로 줌
    viewState.value.offsetX = mouseX - (mouseX - viewState.value.offsetX) * scaleChange
    viewState.value.offsetY = mouseY - (mouseY - viewState.value.offsetY) * scaleChange
    viewState.value.scale = newScale
  }
}

const zoomIn = () => {
  if (viewState.value.scale < 5) {
    viewState.value.scale *= 1.2
  }
}

const zoomOut = () => {
  if (viewState.value.scale > 0.5) {
    viewState.value.scale *= 0.8
  }
}

// 패닝 제어
const startPan = (e) => {
  viewState.value.isPanning = true
  viewState.value.lastX = e.clientX
  viewState.value.lastY = e.clientY
}

const pan = (e) => {
  if (!viewState.value.isPanning) return

  const deltaX = e.clientX - viewState.value.lastX
  const deltaY = e.clientY - viewState.value.lastY

  viewState.value.offsetX += deltaX
  viewState.value.offsetY += deltaY
  viewState.value.lastX = e.clientX
  viewState.value.lastY = e.clientY
}

const stopPan = () => {
  viewState.value.isPanning = false
}

// 뷰 리셋
const resetView = () => {
  viewState.value = {
    scale: 1,
    offsetX: 0,
    offsetY: 0,
    isPanning: false,
    lastX: 0,
    lastY: 0
  }
}

// 윈도우 리사이즈 핸들러
const handleResize = () => {
  if (mapContainer.value && mapCanvas.value) {
    mapCanvas.value.width = mapContainer.value.clientWidth
    mapCanvas.value.height = mapContainer.value.clientHeight
    drawMap()
  }
}

// 라이프사이클 훅
onMounted(() => {
  initCanvas()
  drawMap()
  window.addEventListener('resize', handleResize)
})

onUnmounted(() => {
  window.removeEventListener('resize', handleResize)
})

// props 변경 감지
watch([() => props.mapData, () => props.robotPosition, () => viewState.value], () => {
  drawMap()
}, { deep: true })
</script>

<style scoped>
.map-viewer {
  width: 100%;
  height: 100%;
  position: relative;
  overflow: hidden;
}

canvas {
  width: 100%;
  height: 100%;
  display: block;
}

.map-controls {
  position: absolute;
  right: 10px;
  bottom: 10px;
  display: flex;
  flex-direction: column;
  gap: 5px;
  background: white;
  padding: 5px;
  border-radius: 4px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.control-btn {
  width: 30px;
  height: 30px;
  border: none;
  border-radius: 4px;
  background: white;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #666;
}

.control-btn:hover {
  background: #f5f5f5;
}
</style> 