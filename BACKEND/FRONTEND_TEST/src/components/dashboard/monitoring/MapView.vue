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

      <!-- 선택된 노드들 정보 표시 수정 -->
      <div v-if="selectedNodes.length > 0" class="selected-node-info">
        <h4>선택된 노드 ({{ selectedNodes.length }})</h4>
        <div v-for="(node, index) in selectedNodesInfo" :key="index" class="node-info">
          <p>노드 {{index + 1}}</p>
          <p>X: {{ node.x }}</p>
          <p>Y: {{ node.y }}</p>
        </div>
      </div>

      <div v-if="loading" class="loading-overlay">
        <div class="spinner"></div>
        <span>맵 데이터를 불러오는 중...</span>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, computed } from 'vue'
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

// 선택된 노드들을 배열로 관리하도록 수정
const selectedNodes = ref([])

// 노드 선택 처리 함수 수정
const toggleNodeSelection = (node) => {
  const index = selectedNodes.value.findIndex(n => 
    n.id[0] === node.id[0] && n.id[1] === node.id[1]
  )
  
  if (index === -1) {
    selectedNodes.value.push(node)
  } else {
    selectedNodes.value.splice(index, 1)
  }

  // 차트를 완전히 다시 그리기
  updateChart()
}

// 차트 생성 함수와 업데이트 함수 분리
const createChart = () => {
  if (!chartRef.value) return
  const ctx = chartRef.value.getContext('2d')
  if (!ctx || !mapData.value?.nodes) return

  try {
    if (chartInstance.value) {
      chartInstance.value.destroy()
    }
    
    // 노드 데이터 준비
    const nodeData = mapData.value.nodes
      .filter(node => node && node.id && Array.isArray(node.id) && node.id.length >= 2)
      .map(node => ({
        x: node.id[0],
        y: node.id[1]
      }))

    // 링크 데이터셋 준비
    const linkDatasets = (mapData.value.links || [])
      .filter(link => link && link.source && link.target)
      .map((link, index) => ({
        label: `Link ${index}`,
        data: [
          { x: link.source[0], y: link.source[1] },
          { x: link.target[0], y: link.target[1] }
        ],
        showLine: true,
        borderColor: '#666',
        borderWidth: 1,
        pointRadius: 0,
        fill: false,
        interaction: {
          mode: 'nearest',
          intersect: false
        }
      }))

    chartInstance.value = new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: [
          {
            label: 'Nodes',
            data: nodeData,
            backgroundColor: (context) => {
              if (!context || typeof context.dataIndex === 'undefined' || !mapData.value?.nodes) {
                return '#007bff'
              }
              const index = context.dataIndex
              const node = mapData.value.nodes[index]
              if (!node || !node.id) {
                return '#007bff'
              }
              return selectedNodes.value.some(selectedNode => 
                selectedNode.id[0] === node.id[0] && 
                selectedNode.id[1] === node.id[1]
              ) ? '#ff4081' : '#007bff'
            },
            pointRadius: (context) => {
              if (!context || typeof context.dataIndex === 'undefined' || !mapData.value?.nodes) {
                return 5
              }
              const index = context.dataIndex
              const node = mapData.value.nodes[index]
              if (!node || !node.id) {
                return 5
              }
              return selectedNodes.value.some(selectedNode => 
                selectedNode.id[0] === node.id[0] && 
                selectedNode.id[1] === node.id[1]
              ) ? 8 : 5
            },
            pointHoverRadius: 12,
            interaction: {
              mode: 'point',
              intersect: true
            }
          },
          ...linkDatasets
        ]
      },
      options: {
        animation: {
          duration: 1  // 매우 짧은 애니메이션 (0으로 하면 업데이트가 안될 수 있음)
        },
        responsive: true,
        maintainAspectRatio: false,
        interaction: {
          mode: 'point',
          intersect: true
        },
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
            mode: 'point',
            callbacks: {
              label: (context) => {
                if (!context || !mapData.value?.nodes || typeof context.dataIndex === 'undefined') {
                  return ''
                }
                const node = mapData.value.nodes[context.dataIndex]
                if (!node || !node.id) {
                  return ''
                }
                return `노드 좌표: (${context.parsed.x.toFixed(2)}, ${context.parsed.y.toFixed(2)})`
              }
            }
          },
          zoom: {
            pan: {
              enabled: true,
              mode: 'xy'
            },
            zoom: {
              wheel: {
                enabled: true,
                speed: 0.1
              },
              pinch: {
                enabled: true
              },
              mode: 'xy'
            },
            limits: {
              x: {min: -Infinity, max: Infinity},
              y: {min: -Infinity, max: Infinity}
            }
          }
        },
        onClick: (event, elements) => {
          if (!elements || !mapData.value?.nodes) return
          
          if (elements.length > 0) {
            const element = elements[0]
            if (element.datasetIndex === 0 && 
                typeof element.index !== 'undefined' && 
                element.index < mapData.value.nodes.length) {
              const node = mapData.value.nodes[element.index]
              if (node && node.id) {
                toggleNodeSelection(node)
              }
            }
          }
        }
      }
    })
  } catch (error) {
    console.error('차트 생성 중 오류 발생:', error)
    selectedNodes.value = []
  }
}

// 차트 업데이트 함수 수정
const updateChart = () => {
  if (!chartInstance.value) {
    createChart()
    return
  }

  chartInstance.value.update()
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

// fetchMapData 함수 수정
const fetchMapData = async () => {
  try {
    loading.value = true
    const response = await axios.get('http://localhost:8000/map')
    mapData.value = response.data
    createChart()  // 초기 차트 생성
  } catch (error) {
    console.error('맵 데이터 로딩 실패:', error)
  } finally {
    loading.value = false
  }
}

// onMounted 수정
onMounted(async () => {
  try {
    await fetchMapData()
    window.addEventListener('resize', () => {
      createChart()  // 리사이즈 시 차트 재생성
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

// 선택된 노드들의 정보를 표시하기 위한 computed 속성
const selectedNodesInfo = computed(() => {
  return selectedNodes.value.map(node => ({
    x: (node.id[0] || 0).toFixed(2),
    y: (node.id[1] || 0).toFixed(2)
  }))
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

.selected-node-info {
  position: absolute;
  top: 20px;
  right: 20px;
  background: white;
  padding: 15px;
  border-radius: 4px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.selected-node-info h4 {
  margin: 0 0 10px 0;
  color: #333;
}

.selected-node-info p {
  margin: 5px 0;
  color: #666;
}

.node-info {
  margin-bottom: 10px;
  padding-bottom: 10px;
  border-bottom: 1px solid #eee;
}

.node-info:last-child {
  margin-bottom: 0;
  padding-bottom: 0;
  border-bottom: none;
}
</style> 