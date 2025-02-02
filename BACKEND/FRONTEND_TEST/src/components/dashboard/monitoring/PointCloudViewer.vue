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

    <div class="viewer-container" ref="container"></div>

    <div v-if="loading" class="loading-overlay">
      <div class="loading-spinner"></div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'
import webSocketService from '@/services/webSocketService'

const container = ref(null)
const selectedRobot = ref('')
const loading = ref(false)
const isConnected = ref(false)

// Three.js 변수
let scene, camera, renderer, controls
let pointCloud

// 임시 로봇 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' }
])

// 전역 변수 선언
let ws = null  // 웹소켓 연결 객체를 전역으로 관리
const connectWebSocket = () => {
  try {
    webSocketService.connect('ws://localhost:8000/api/v1/lidar/ws')
    .then(() => {
      webSocketService.subscribe('lidar_points', (data) => {
        if (data && data.points && data.points.length > 0) {
          console.log('Received points:', data.points.length)
          updatePointCloud(data.points)
        }
      }, '/api/v1/lidar/ws')
    })
    .catch(error => {
      console.error('라이다 웹소켓 연결 실패:', error)
    })
  } catch (error) {
    console.error('라이다 웹소켓 연결 에러:', error)
  }
}

// Three.js 초기화
const initThree = () => {
  scene = new THREE.Scene()
  scene.background = new THREE.Color(0x1a1a1a)
  
  camera = new THREE.PerspectiveCamera(
    75,
    container.value.clientWidth / container.value.clientHeight,
    0.1,
    1000
  )
  camera.position.set(5, 5, 5)
  camera.lookAt(0, 0, 0)
  
  renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setSize(container.value.clientWidth, container.value.clientHeight)
  container.value.appendChild(renderer.domElement)
  
  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  
  // 그리드 헬퍼 추가
  const gridHelper = new THREE.GridHelper(20, 20)
  scene.add(gridHelper)
  
  // 축 헬퍼 추가
  const axesHelper = new THREE.AxesHelper(5)
  scene.add(axesHelper)
  
  // 포인트 클라우드 초기화
  const geometry = new THREE.BufferGeometry()
  const material = new THREE.PointsMaterial({
    size: 0.1,  // 각 포인트의 크기
    color: 0x2196f3,  // 포인트 색상 (파란색)
    transparent: true,  // 투명도 활성화
    opacity: 0.8  // 투명도 값
  })
  
  // 초기 빈 포인트 버퍼 생성
  const positions = new Float32Array(3)
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  
  // 포인트 클라우드 메쉬 생성 및 씬에 추가
  pointCloud = new THREE.Points(geometry, material)
  scene.add(pointCloud)
  
  animate()
}

// 포인트 클라우드 업데이트
const updatePointCloud = (points) => {
  if (!pointCloud || !points.length) return
  
  try {
    // 새로운 포인트 위치 배열 생성
    const positions = new Float32Array(points.length * 3)
    
    // 각 포인트의 위치 데이터 설정
    points.forEach((point, i) => {
      positions[i * 3] = point.x
      positions[i * 3 + 1] = point.y
      positions[i * 3 + 2] = point.z
    })
    
    // 새로운 geometry 생성 및 포인트 위치 설정
    const newGeometry = new THREE.BufferGeometry()
    newGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    
    // 기존 geometry 정리 및 새것으로 교체
    if (pointCloud.geometry) {
      pointCloud.geometry.dispose()  // 메모리 누수 방지
    }
    pointCloud.geometry = newGeometry
    
    // 바운딩 스피어 계산 (카메라 시점 조절용)
    pointCloud.geometry.computeBoundingSphere()
    
  } catch (error) {
    console.error('Error updating point cloud:', error)
  }
}

// 애니메이션 루프
const animate = () => {
  requestAnimationFrame(animate)
  if (controls) controls.update()
  if (renderer && scene && camera) {
    renderer.render(scene, camera)
  }
}

// 윈도우 리사이즈 핸들러
const handleResize = () => {
  if (!container.value || !camera || !renderer) return
  
  camera.aspect = container.value.clientWidth / container.value.clientHeight
  camera.updateProjectionMatrix()
  renderer.setSize(container.value.clientWidth, container.value.clientHeight)
}

onMounted(() => {
  try {
    initThree()
    connectWebSocket()
    window.addEventListener('resize', handleResize)
  } catch (error) {
    console.error('라이다 뷰어 초기화 에러:', error);
  }
})

onUnmounted(() => {
  window.removeEventListener('resize', handleResize)
  if (renderer) {
    renderer.dispose()
  }
  if (controls) {
    controls.dispose()
  }
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