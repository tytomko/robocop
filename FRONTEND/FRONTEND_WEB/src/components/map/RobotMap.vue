<template>
  <div class="flex flex-col h-screen bg-white">
    <div class="flex-1 relative p-5">
      <div class="flex space-x-3 mb-4">
        <button 
          class="control-btn"
          :class="{ 'bg-blue-600 text-white': selectedNodes.length === 1 }"
          :disabled="selectedNodes.length !== 1"
          @click="handleNavigate"
        >
          <i class="mdi mdi-navigation text-lg"></i>
          이동
        </button>
        <button 
          class="control-btn"
          :class="{ 'bg-green-600 text-white': selectedNodes.length >= 2 }"
          :disabled="selectedNodes.length < 2"
          @click="handlePatrol"
        >
          <i class="mdi mdi-routes text-lg"></i>
          순찰
        </button>
        <button 
          class="control-btn bg-red-500 text-white"
          @click="resetSelection"
        >
          <i class="mdi mdi-refresh text-lg"></i>
          리셋
        </button>
        <button 
          class="control-btn bg-orange-500 text-white"
          @click="handleTempStop"
        >
          <i class="mdi mdi-pause-circle text-lg"></i>
          일시정지
        </button>
      </div>

      <v-chart 
        class="w-full h-full min-h-[500px]" 
        :option="chartOption" 
        ref="chartRef" 
        autoresize
        @click="handleNodeClick"
      />

      <div v-if="selectedNodes.length > 0" class="absolute top-5 right-5 bg-white shadow-md p-4 rounded-lg border">
        <h4 class="text-lg font-semibold text-gray-800 mb-3">
          선택된 노드 ({{ selectedNodes.length }})
        </h4>
        <div v-for="(node, index) in selectedNodesInfo" :key="index" class="p-2 border rounded-md bg-gray-100">
          <p class="font-semibold">노드 {{ index + 1 }}</p>
          <p class="text-gray-600 text-sm">X: {{ node.x }}</p>
          <p class="text-gray-600 text-sm">Y: {{ node.y }}</p>
        </div>
      </div>

      <div v-if="loading" class="absolute inset-0 flex flex-col items-center justify-center bg-white bg-opacity-90">
        <div class="w-10 h-10 border-4 border-gray-300 border-t-blue-500 rounded-full animate-spin"></div>
        <span class="mt-3 text-gray-700">맵 데이터를 불러오는 중...</span>
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
      data: mapData.value.nodes.map(node => [node.id[0], node.id[1]]),
      symbolSize: (value) => {
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
              data: mapData.value.nodes.map(node => [node.id[0], node.id[1]]),
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
    const response = await axios.get('https://robocop-backend-app.fly.dev/map')
    mapData.value = { nodes: response.data.nodes, links: response.data.links }
  } catch (error) {
    console.error('맵 데이터 로딩 실패:', error)
  } finally {
    loading.value = false
  }
}

// 선택된 노드들 정보
const selectedNodesInfo = computed(() => {
  return selectedNodes.value.map(node => ({ x: node.id[0].toFixed(2), y: node.id[1].toFixed(2) }))
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
    
    await axios.post('https://robocop-backend-app.fly.dev/api/v1/robot_1/call-service/navigate', {
      goal
    })
    
  } catch (error) {
    console.error('Navigation request failed:', error)
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
    
    await axios.post('https://robocop-backend-app.fly.dev/api/v1/robot_1/call-service/patrol', {
      goals
    })
    
  } catch (error) {
    console.error('Patrol request failed:', error)
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

// E-stop 요청 처리
const handleEstop = async () => {
  try {
    await axios.post('http://localhost:8000/api/v1/robot_1/call-service/estop')
  } catch (error) {
    console.error('E-stop request failed:', error)
  }
}

// 일시정지 요청 처리
const handleTempStop = async () => {
  try {
    await axios.post('ttps://robocop-backend-app.fly.dev/api/v1/robot_1/call-service/temp-stop')
  } catch (error) {
    console.error('Temporary stop request failed:', error)
  }
}

onMounted(async () => {
  await fetchMapData()
})
</script>

<style scoped>
.control-btn {
  @apply flex items-center gap-2 px-4 py-2 rounded-lg text-gray-700 bg-gray-200 hover:bg-gray-300 transition-all disabled:opacity-50 disabled:cursor-not-allowed;
}
</style>