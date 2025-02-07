<template>
  <div class="map-view">
    <div class="map-header">
      <h3>맵 뷰</h3>
      <div class="map-controls">
        <button class="control-btn" @click="resetZoom">
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
      
      <canvas ref="chartRef"></canvas>

      <div v-if="loading" class="loading-overlay">
        <div class="spinner"></div>
        <span>맵 데이터를 불러오는 중...</span>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue'
import axios from 'axios'
import Chart from 'chart.js/auto'
import zoomPlugin from 'chartjs-plugin-zoom'

// Chart.js에 zoom 플러그인 등록
Chart.register(zoomPlugin)

const chartRef = ref(null)
const chartInstance = ref(null)
const selectedRobot = ref(localStorage.getItem('selectedRobot') || '')
const loading = ref(true)
const mapData = ref({
  nodes: [],
  links: [],
  directed: false,
  multigraph: false,
  graph: {}
})

// 임시 로봇 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' },
  { id: 'robot3', name: 'Robot 3' }
])

// 차트 생성/업데이트 함수
const updateChart = () => {
  if (!chartRef.value) return
  const ctx = chartRef.value.getContext('2d')
  if (!ctx) return

  try {
    if (chartInstance.value) {
      chartInstance.value.destroy()
    }
    
    // 노드 데이터 준비
    const nodeData = mapData.value.nodes.map(node => ({
      x: node.id[0],
      y: node.id[1]
    }))

    // 링크 데이터셋 준비
    const linkDatasets = mapData.value.links.map((link, index) => ({
      label: `Link ${index}`,
      data: [
        { x: link.source[0], y: link.source[1] },
        { x: link.target[0], y: link.target[1] }
      ],
      showLine: true,
      borderColor: '#666',
      borderWidth: 1,
      pointRadius: 0,
      fill: false
    }))

    chartInstance.value = new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: [
          {
            label: 'Nodes',
            data: nodeData,
            backgroundColor: '#007bff',
            pointRadius: 5,
            pointHoverRadius: 8
          },
          ...linkDatasets
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: {
            type: 'linear',
            position: 'bottom',
            grid: {
              color: '#ddd'
            }
          },
          y: {
            type: 'linear',
            grid: {
              color: '#ddd'
            }
          }
        },
        plugins: {
          legend: {
            display: false
          },
          tooltip: {
            enabled: true,
            callbacks: {
              label: (context) => {
                return `(${context.parsed.x.toFixed(2)}, ${context.parsed.y.toFixed(2)})`
              }
            }
          },
          zoom: {
            pan: {  // 패닝(이동) 관련 설정
              enabled: true,
              mode: 'xy',
              threshold: 10
            },
            zoom: {   // 확대, 축소 관련 설정
              wheel: {
                enabled: true,
                speed: 0.1
              },
              pinch: {
                enabled: true
              },
              mode: 'xy',
              drag: {
                enabled: false
              }
            },
            limits: {
              x: {min: 'original', max: 'original'},
              y: {min: 'original', max: 'original'}
            }
          }
        }
      }
    })
  } catch (error) {
    console.error('차트 업데이트 중 오류 발생:', error)
  }
}

// 줌 리셋 함수
const resetZoom = () => {
  if (chartInstance.value) {
    chartInstance.value.resetZoom()
  }
}

// 로봇 선택 처리
const handleRobotSelection = () => {
  if (selectedRobot.value) {
    localStorage.setItem('selectedRobot', selectedRobot.value)
  } else {
    localStorage.removeItem('selectedRobot')
  }
}

// 맵 데이터 가져오기
const fetchMapData = async () => {
  try {
    loading.value = true
    const response = await axios.get('http://localhost:8000/map')
    mapData.value = response.data
    updateChart()
  } catch (error) {
    console.error('맵 데이터 로딩 실패:', error)
  } finally {
    loading.value = false
  }
}

// 컴포넌트 마운트 시 데이터 로드
onMounted(async () => {
  try {
    await fetchMapData()
    window.addEventListener('resize', () => {
      requestAnimationFrame(updateChart)
    })
  } catch (error) {
    console.error('컴포넌트 마운트 중 오류 발생:', error)
  }
})

// 컴포넌트 언마운트 시 정리
onUnmounted(() => {
  if (chartInstance.value) {
    chartInstance.value.destroy()
  }
  window.removeEventListener('resize', updateChart)
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

.map-container {
  flex: 1;
  position: relative;
  padding: 20px;
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

canvas {
  width: 100% !important;
  height: 100% !important;
  cursor: grab; /* 마우스 커서 스타일 변경 */
}

canvas:active {
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