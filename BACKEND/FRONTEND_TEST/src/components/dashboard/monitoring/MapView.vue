<template>
  <div class="map-view">
    <div class="map-header">
      <h3>맵 뷰</h3>
      <div class="map-controls">
        <button 
          :class="['control-btn', {'active': selectedNodes.length === 1}]"
          :disabled="selectedNodes.length !== 1"
          @click="handleNavigate"
        >
          <span class="icon">
            <i class="mdi mdi-navigation"></i>
          </span>
          이동
        </button>
        <button 
          :class="['control-btn', {'active': selectedNodes.length >= 2}]"
          :disabled="selectedNodes.length < 2"
          @click="handlePatrol"
        >
          <span class="icon">
            <i class="mdi mdi-routes"></i>
          </span>
          순찰
        </button>
        <button 
          class="control-btn stop-btn"
          @click="handleEstop"
        >
          <span class="icon">
            <i class="mdi mdi-stop-circle"></i>
          </span>
          정지
        </button>
        <button 
          class="control-btn reset-btn"
          @click="resetSelection"
        >
          <span class="icon">
            <i class="mdi mdi-refresh"></i>
          </span>
          리셋
        </button>
        <button 
          class="control-btn pause-btn"
          @click="handleTempStop"
        >
          <span class="icon">
            <i class="mdi mdi-pause-circle"></i>
          </span>
          일시정지
        </button>
        <button 
          class="control-btn play-btn"
          @click="handleResume"
        >
          <span class="icon">
            <i class="mdi mdi-play-circle"></i>
          </span>
          재생
        </button>
        <button 
          class="control-btn wait-btn"
          @click="handleWaiting"
        >
          <span class="icon">
            <i class="mdi mdi-timer"></i>
          </span>
          대기
        </button>
        <button 
          class="control-btn manual-btn"
          @click="handleManual"
        >
          <span class="icon">
            <i class="mdi mdi-gamepad"></i>
          </span>
          매뉴얼
        </button>
      </div>
    </div>

    <div class="map-container">
      <v-chart 
        class="chart" 
        :option="chartOption" 
        ref="chartRef" 
        autoresize
        @click="handleNodeClick"
      />

      <!-- 선택된 노드들 정보 표시 -->
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
import { ref, computed, onMounted } from 'vue'
import { use } from 'echarts/core'
import { CanvasRenderer } from 'echarts/renderers'
import { 
  GraphicComponent, 
  GridComponent, 
  TooltipComponent,
  DataZoomComponent,
  ToolboxComponent
} from 'echarts/components'
import { ScatterChart, LinesChart } from 'echarts/charts'
import VChart from 'vue-echarts'
import axios from 'axios'

// ECharts 컴포넌트 등록
use([
  CanvasRenderer,
  GraphicComponent,
  GridComponent,
  TooltipComponent,
  DataZoomComponent,
  ToolboxComponent,
  ScatterChart,
  LinesChart
])

const chartRef = ref(null)
const loading = ref(true)
const selectedNodes = ref([])
const mapData = ref({
  nodes: [],
  links: []
})

// 차트 옵션 computed 속성
const chartOption = computed(() => ({
  animation: false,
  tooltip: {
    trigger: 'item',
    formatter: (params) => {
      if (params.componentSubType === 'scatter') {
        return `좌표: (${params.data[0].toFixed(2)}, ${params.data[1].toFixed(2)})`
      }
      return ''
    }
  },
  dataZoom: [
    {
      type: 'inside',
      xAxisIndex: [0],
      yAxisIndex: [0],
      minSpan: 1,
      maxSpan: 100
    },
    {
      type: 'inside',
      xAxisIndex: [0],
      yAxisIndex: [0],
      minSpan: 1,
      maxSpan: 100
    }
  ],
  xAxis: {
    type: 'value',
    scale: true,
    axisLine: { onZero: false }
  },
  yAxis: {
    type: 'value',
    scale: true,
    axisLine: { onZero: false }
  },
  series: [
    // 엣지(연결선) 표시
    {
      type: 'lines',
      coordinateSystem: 'cartesian2d',
      data: mapData.value.links.map(link => ({
        coords: [
          [link.source[0], link.source[1]],
          [link.target[0], link.target[1]]
        ]
      })),
      lineStyle: {
        color: '#666',
        width: 1,
        opacity: 0.6
      },
      zlevel: 1
    },
    // 노드 표시
    {
      type: 'scatter',
      data: mapData.value.nodes.map(node => node.id),
      symbolSize: (value, params) => {
        // 선택된 노드는 더 크게 표시
        return selectedNodes.value.some(selected => 
          selected.id[0] === value[0] && selected.id[1] === value[1]
        ) ? 15 : 8
      },
      itemStyle: {
        color: (params) => {
          const node = mapData.value.nodes[params.dataIndex]
          // 선택된 노드는 다른 색상으로 표시
          return selectedNodes.value.some(selected => 
            selected.id[0] === node.id[0] && selected.id[1] === node.id[1]
          ) ? '#ff4081' : '#007bff'
        }
      },
      emphasis: {
        scale: 1.5,  // 마우스 오버 시 크기 증가
        itemStyle: {
          shadowBlur: 10,
          shadowColor: 'rgba(0, 0, 0, 0.3)'
        }
      },
      zlevel: 2
    }
  ]
}))

// 노드 선택 처리
const handleNodeClick = (params) => {
  if (params.componentSubType === 'scatter') {
    const clickedNode = mapData.value.nodes[params.dataIndex]
    
    if (clickedNode) {
      const index = selectedNodes.value.findIndex(node => 
        node.id[0] === clickedNode.id[0] && node.id[1] === clickedNode.id[1]
      )
      
      if (index === -1) {
        selectedNodes.value.push(clickedNode)
      } else {
        selectedNodes.value.splice(index, 1)
      }

      // 차트 즉시 업데이트
      if (chartRef.value) {
        chartRef.value.setOption({
          series: [
            // 엣지(연결선) 시리즈는 유지
            chartOption.value.series[0],
            {
              type: 'scatter',
              data: mapData.value.nodes.map(node => node.id),
              symbolSize: (value) => {
                return selectedNodes.value.some(selected => 
                  selected.id[0] === value[0] && selected.id[1] === value[1]
                ) ? 15 : 8
              },
              itemStyle: {
                color: (params) => {
                  const node = mapData.value.nodes[params.dataIndex]
                  return selectedNodes.value.some(selected => 
                    selected.id[0] === node.id[0] && selected.id[1] === node.id[1]
                  ) ? '#ff4081' : '#007bff'
                }
              },
              emphasis: {
                scale: 1.5,
                itemStyle: {
                  shadowBlur: 10,
                  shadowColor: 'rgba(0, 0, 0, 0.3)'
                }
              },
              zlevel: 2
            }
          ]
        })
      }
    }
  }
}

// 맵 데이터 가져오기
const fetchMapData = async () => {
  try {
    loading.value = true
    const response = await axios.get('http://localhost:8000/map')
    mapData.value = {
      nodes: response.data.nodes,
      links: response.data.links
    }
  } catch (error) {
    console.error('맵 데이터 로딩 실패:', error)
  } finally {
    loading.value = false
  }
}

// 선택된 노드들 정보
const selectedNodesInfo = computed(() => {
  return selectedNodes.value.map(node => ({
    x: node.id[0].toFixed(2),
    y: node.id[1].toFixed(2)
  }))
})

// 줌 리셋 함수 추가
const resetZoom = () => {
  if (chartRef.value) {
    chartRef.value.setOption({
      dataZoom: [
        {
          start: 0,
          end: 100
        },
        {
          start: 0,
          end: 100
        }
      ]
    })
  }
}

// 네비게이션 요청 처리
const handleNavigate = async () => {
  if (selectedNodes.value.length !== 1) return
  
  try {
    const goal = {
      x: selectedNodes.value[0].id[0],
      y: selectedNodes.value[0].id[1],
      theta: 0.0
    }
    
    await axios.post('http://localhost:8000/api/v1/ros-publisher/call-service/navigate', {
      goal
    })
    
    // 성공 메시지 표시 로직 추가
  } catch (error) {
    console.error('Navigation request failed:', error)
    // 에러 메시지 표시 로직 추가
  }
}

// 순찰 요청 처리
const handlePatrol = async () => {
  if (selectedNodes.value.length < 2) return
  
  try {
    const goals = selectedNodes.value.map(node => ({
      x: node.id[0],
      y: node.id[1],
      theta: 0.0
    }))
    
    await axios.post('http://localhost:8000/api/v1/ros-publisher/call-service/patrol', {
      goals
    })
    
    // 성공 메시지 표시 로직 추가
  } catch (error) {
    console.error('Patrol request failed:', error)
    // 에러 메시지 표시 로직 추가
  }
}

// 선택 초기화
const resetSelection = () => {
  selectedNodes.value = []
  if (chartRef.value) {
    chartRef.value.setOption({
      series: chartOption.value.series
    })
  }
}

// E-stop 요청 처리 함수 추가
const handleEstop = async () => {
  try {
    await axios.post('http://localhost:8000/api/v1/ros-publisher/call-service/estop')
    // 성공 메시지 표시 로직 추가
  } catch (error) {
    console.error('E-stop request failed:', error)
    // 에러 메시지 표시 로직 추가
  }
}

// 일시정지 요청 처리 함수 추가
const handleTempStop = async () => {
  try {
    await axios.post('http://localhost:8000/api/v1/ros-publisher/call-service/temp-stop')
    // 성공 메시지 표시 로직 추가
  } catch (error) {
    console.error('Temporary stop request failed:', error)
    // 에러 메시지 표시 로직 추가
  }
}

// 재생 요청 처리 함수 추가
const handleResume = async () => {
  try {
    await axios.post('http://localhost:8000/api/v1/ros-publisher/call-service/resume')
    // 성공 메시지 표시 로직 추가
  } catch (error) {
    console.error('Resume request failed:', error)
    // 에러 메시지 표시 로직 추가
  }
}

// 대기 요청 처리 함수 추가
const handleWaiting = async () => {
  try {
    await axios.post('http://localhost:8000/api/v1/ros-publisher/call-service/waiting')
    // 성공 메시지 표시 로직 추가
  } catch (error) {
    console.error('Waiting request failed:', error)
    // 에러 메시지 표시 로직 추가
  }
}

// 매뉴얼 모드 요청 처리 함수 추가
const handleManual = async () => {
  try {
    await axios.post('http://localhost:8000/api/v1/ros-publisher/call-service/manual')
    // 성공 메시지 표시 로직 추가
  } catch (error) {
    console.error('Manual mode request failed:', error)
    // 에러 메시지 표시 로직 추가
  }
}

onMounted(async () => {
  await fetchMapData()
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
  padding: 16px 24px;
  border-bottom: 1px solid #eee;
  background-color: #f8f9fa;
}

.map-header h3 {
  margin: 0;
  color: #2c3e50;
  font-size: 1.5rem;
  font-weight: 600;
}

.map-container {
  flex: 1;
  position: relative;
  padding: 20px;
}

.chart {
  width: 100%;
  height: 100%;
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
  padding: 20px;
  border-radius: 12px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  max-width: 300px;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.2);
}

.selected-node-info h4 {
  margin: 0 0 15px 0;
  color: #2c3e50;
  font-size: 1.1rem;
  font-weight: 600;
}

.node-info {
  background: #f8f9fa;
  padding: 12px;
  margin-bottom: 12px;
  border-radius: 8px;
  border: 1px solid #eee;
}

.node-info p {
  margin: 4px 0;
  color: #4a5568;
  font-size: 0.95rem;
}

.node-info p:first-child {
  color: #2c3e50;
  font-weight: 500;
  margin-bottom: 8px;
}

.map-controls {
  display: flex;
  gap: 12px;
}

.control-btn {
  min-width: 100px;
  height: 40px;
  padding: 0 20px;
  border: none;
  border-radius: 8px;
  background-color: #f0f0f0;
  color: #666;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  transition: all 0.2s ease;
}

.control-btn:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.control-btn.active {
  background-color: #1976d2;
  color: white;
}

.control-btn.active:hover {
  background-color: #1565c0;
}

.control-btn.reset-btn {
  background-color: #ef5350;
  color: white;
}

.control-btn.reset-btn:hover {
  background-color: #e53935;
}

.control-btn.stop-btn {
  background-color: #d32f2f;
  color: white;
}

.control-btn.stop-btn:hover {
  background-color: #b71c1c;
}

.control-btn.pause-btn {
  background-color: #fb8c00;  /* 주황색 계열 */
  color: white;
}

.control-btn.pause-btn:hover {
  background-color: #f57c00;
}

.control-btn.play-btn {
  background-color: #4caf50;  /* 초록색 계열 */
  color: white;
}

.control-btn.play-btn:hover {
  background-color: #43a047;
}

.control-btn.wait-btn {
  background-color: #9c27b0;  /* 보라색 계열 */
  color: white;
}

.control-btn.wait-btn:hover {
  background-color: #7b1fa2;
}

.control-btn.manual-btn {
  background-color: #2196f3;  /* 파란색 계열 */
  color: white;
}

.control-btn.manual-btn:hover {
  background-color: #1976d2;
}

.control-btn:hover:not(:disabled) {
  transform: translateY(-1px);
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.icon {
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 18px;
}
</style> 