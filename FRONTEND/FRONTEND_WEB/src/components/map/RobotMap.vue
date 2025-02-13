<template>
  <div class="flex flex-col h-screen bg-white">
    <div class="flex-1 relative p-1">
      <!-- (버튼 영역 사라짐) -->

      <v-chart
        class="w-5/6 h-[600px] min-h-[300px] mx-auto"
        :option="chartOption"
        ref="chartRef"
        autoresize
        @click="handleNodeClick"
      />


      <!-- 선택된 노드들을 표시하는 컴포넌트 -->
      <SelectedNodes :selectedNodes="selectedNodesInfo" />

      <!-- 로딩 표시 -->
      <div v-if="loading" class="absolute inset-0 flex flex-col items-center justify-center bg-white bg-opacity-90">
        <div class="w-10 h-10 border-4 border-gray-300 border-t-blue-500 rounded-full animate-spin"></div>
        <span class="mt-3 text-gray-700">맵 데이터를 불러오는 중...</span>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue'
import axios from 'axios'
import { use } from 'echarts/core'
import { CanvasRenderer } from 'echarts/renderers'
import { GraphicComponent, GridComponent, TooltipComponent, DataZoomComponent, ToolboxComponent } from 'echarts/components'
import { ScatterChart, LinesChart } from 'echarts/charts'
import VChart from 'vue-echarts'
import SelectedNodes from '@/components/map/SelectedNodes.vue'

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

// 이벤트 정의
const emit = defineEmits(['selectedNodesChange'])

// 네비게이션 요청 처리
async function handleNavigate() {
  if (selectedNodes.value.length !== 1) return
  try {
    const goal = {
      x: selectedNodes.value[0].id[0],
      y: selectedNodes.value[0].id[1],
      theta: 0.0
    }
    await axios.post('https://robocop-backend-app.fly.dev/api/v1/robot_1/call-service/navigate', { goal })
    console.log('[RobotMap] 이동 명령 전송 완료')
  } catch (error) {
    console.error('Navigation request failed:', error)
  }
}

async function handlePatrol() {
  if (selectedNodes.value.length < 2) return
  try {
    const goals = selectedNodes.value.map(node => ({
      x: node.id[0],
      y: node.id[1],
      theta: 0.0
    }))
    await axios.post('https://robocop-backend-app.fly.dev/api/v1/robot_1/call-service/patrol', { goals })
    console.log('[RobotMap] 순찰 명령 전송 완료')
  } catch (error) {
    console.error('Patrol request failed:', error)
  }
}

function resetSelection() {
  selectedNodes.value = []
  if (chartRef.value) {
    chartRef.value.setOption({
      series: chartOption.value.series
    })
  }
}

async function handleTempStop() {
  try {
    await axios.post('https://robocop-backend-app.fly.dev/api/v1/robot_1/call-service/temp-stop')
    console.log('[RobotMap] 일시정지 요청 완료')
  } catch (error) {
    console.error('Temporary stop request failed:', error)
  }
}

// expose로 부모 컴포넌트가 직접 접근 가능
defineExpose({
  handleNavigate,
  handlePatrol,
  resetSelection,
  handleTempStop
})

const chartRef = ref(null)
const loading = ref(true)
const mapData = ref({ nodes: [], links: [] })
const selectedNodes = ref([])

// 선택된 노드 좌표(소수점 2자리)만 추출
const selectedNodesInfo = computed(() => {
  return selectedNodes.value.map(node => ({
    x: node.id[0].toFixed(2),
    y: node.id[1].toFixed(2)
  }))
})

// 선택된 노드 바뀔 때마다 부모에게 알림
watch(selectedNodes, (newVal) => {
  // 디버그 확인용
  console.log('RobotMap: selectedNodes changed:', newVal)
  // 부모에게 이벤트 전송
  emit('selectedNodesChange', newVal)
})


// 차트 옵션
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

/** 노드 클릭 시 선택/해제 */
function handleNodeClick(params) {
  if (params.componentSubType === 'scatter') {
    const clickedNode = mapData.value.nodes[params.dataIndex]
    if (!clickedNode) return

    const index = selectedNodes.value.findIndex(n =>
      n.id[0] === clickedNode.id[0] && n.id[1] === clickedNode.id[1]
    )
    if (index === -1) {
      selectedNodes.value.push(clickedNode)
    } else {
      selectedNodes.value.splice(index, 1)
    }

    // 심볼/색상 즉시 반영
    if (chartRef.value) {
      chartRef.value.setOption({
        series: [
          chartOption.value.series[0], // lines
          {
            ...chartOption.value.series[1], // scatter
            data: mapData.value.nodes.map(node => [node.id[0], node.id[1]]),
            symbolSize: (value) =>
              selectedNodes.value.some(sel => sel.id[0] === value[0] && sel.id[1] === value[1])
                ? 15
                : 8,
            itemStyle: {
              color: (p) => {
                const node = mapData.value.nodes[p.dataIndex]
                return selectedNodes.value.some(sel =>
                  sel.id[0] === node.id[0] && sel.id[1] === node.id[1]
                ) ? '#ff4081' : '#007bff'
              }
            }
          }
        ]
      })
    }
  }
} 

/** 로딩 후 맵 데이터 가져오기 */
async function fetchMapData() {
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
onMounted(() => {
  fetchMapData()
})
</script>
