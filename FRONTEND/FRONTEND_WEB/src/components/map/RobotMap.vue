<template>
  <div class="flex flex-col h-screen bg-white">
    <div class="flex-1 relative p-1">
      <div class="relative w-full h-[550px] min-h-[300px] mx-auto" ref="containerRef">
        <v-chart
          class="absolute inset-0 w-full h-full"
          :option="chartOption"
          ref="chartRef"
          autoresize
          @click="handleNodeClick"
        />
      </div>

      <SelectedNodes 
        v-if="props.showSelectedNodes" 
        :selectedNodes="selectedNodesInfo" 
      />

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
import { GraphicComponent, GridComponent, TooltipComponent, ToolboxComponent } from 'echarts/components'
import { ScatterChart, LinesChart } from 'echarts/charts'
import VChart from 'vue-echarts'
import SelectedNodes from '@/components/map/SelectedNodes.vue'
import { useRobotsStore } from '@/stores/robots'
import { useRobotCommandsStore } from '@/stores/robotCommands'

use([
  CanvasRenderer,
  GraphicComponent,
  GridComponent,
  TooltipComponent,
  ToolboxComponent,
  ScatterChart,
  LinesChart
])

const robotsStore = useRobotsStore()
const robotCommandsStore = useRobotCommandsStore()
const emit = defineEmits(['selectedNodesChange'])
const containerRef = ref(null)

const props = defineProps({
  showSelectedNodes: {
    type: Boolean,
    default: true
  },
  robot: {
    type: Object,
    required: false,
    default: () => null
  }
})

const currentRobotSeq = computed(() => {
  if (props.robot) {
    return props.robot.seq
  }

  const selectedRobotSeq = robotsStore.selectedRobot
  if (selectedRobotSeq) {
    const selectedRobot = robotsStore.registered_robots.find(
      robot => robot.seq === selectedRobotSeq
    )
    return selectedRobot?.seq
  }
  return null
})

async function handleNavigate() {
  await robotCommandsStore.navigateCommand(selectedNodes.value, currentRobotSeq.value)
}

async function handlePatrol() {
  await robotCommandsStore.patrolCommand(selectedNodes.value, currentRobotSeq.value)
}

async function resetSelection() {
  selectedNodes.value = await robotCommandsStore.resetSelectionCommand(currentRobotSeq.value)
  updateChartSeries()
}

async function handleTempStop() {
  await robotCommandsStore.tempStopCommand(currentRobotSeq.value)
}

async function handleResume() {
  await robotCommandsStore.resumeCommand(currentRobotSeq.value)
}

defineExpose({
  handleNavigate,
  handlePatrol,
  resetSelection,
  handleTempStop,
  handleResume
})

const chartRef = ref(null)
const loading = ref(true)
const mapData = ref({ nodes: [], links: [] })
const selectedNodes = ref([])
const imageWidth = ref(800)
const imageHeight = ref(500)

// resize observer 설정
onMounted(() => {
  const resizeObserver = new ResizeObserver((entries) => {
    for (const entry of entries) {
      const { width, height } = entry.contentRect
      // 컨테이너 크기에 맞춰 이미지 크기 조정
      imageWidth.value = width * 0.9  // 여백을 위해 90%만 사용
      imageHeight.value = height * 0.95
      
      if (chartRef.value) {
        chartRef.value.resize()
      }
    }
  })

  if (containerRef.value) {
    resizeObserver.observe(containerRef.value)
  }
})

const selectedNodesInfo = computed(() => {
  return selectedNodes.value.map(node => ({
    x: node.id[0].toFixed(2),
    y: node.id[1].toFixed(2)
  }))
})

function updateChartSeries() {
  if (chartRef.value) {
    chartRef.value.setOption({
      series: [
        chartOption.value.series[0],
        {
          ...chartOption.value.series[1],
          data: mapData.value.nodes.map(node => [node.id[0], node.id[1]]),
          symbolSize: (value) =>
            selectedNodes.value.some(sel => sel.id[0] === value[0] && sel.id[1] === value[1])
              ? 15
              : 8,
          itemStyle: {
            color: (params) => {
              const node = mapData.value.nodes[params.dataIndex]
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

const chartOption = computed(() => ({
  animation: false,
  backgroundColor: '#fff',
  grid: {
    left: '5%',
    right: '5%',
    top: '5%',
    bottom: '5%',
    containLabel: true
  },
  graphic: [
    {
      type: 'image',
      id: 'backgroundImage',
      z: -10,
      left: 'center',
      top: 'middle',
      style: {
        image: '/images/row-map.png',
        width: imageWidth.value * 1.01,
        height: imageHeight.value * 1.1
      }
    }
  ],
  tooltip: {
    trigger: 'item',
    formatter: (params) => {
      if (params.componentSubType === 'scatter') {
        return `좌표: (${params.data[0].toFixed(2)}, ${params.data[1].toFixed(2)})`
      }
      return ''
    }
  },
  xAxis: {
    type: 'value',
    scale: true,
    axisLine: { show: false },
    splitLine: { show: false },
    axisTick: { show: false },
    axisLabel: { show: false }
  },
  yAxis: {
    type: 'value',
    scale: true,
    axisLine: { show: false },
    splitLine: { show: false },
    axisTick: { show: false },
    axisLabel: { show: false }
  },
  series: [
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
        color: '#2196F3',
        width: 2,
        opacity: 0.8
      },
      zlevel: 1
    },
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
}))

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

    if (chartRef.value) {
      chartRef.value.setOption({
        series: [
          chartOption.value.series[0],
          {
            ...chartOption.value.series[1],
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

    console.log('Selected nodes updated:', selectedNodes.value)
    emit('selectedNodesChange', selectedNodes.value)
  }
}

watch(selectedNodes, (newVal) => {
  console.log("🔵 Selected nodes updated:", newVal)
  if (chartRef.value) {
    chartRef.value.setOption({
      series: [
        chartOption.value.series[0],
        {
          ...chartOption.value.series[1],
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
})

watch(() => props.robot, (newRobot) => {
  if (newRobot) {
    console.log('Robot changed in RobotMap:', newRobot)
    selectedNodes.value = []
  }
}, { deep: true })

async function fetchMapData() {
  try {
    loading.value = true
    const response = await axios.get('https://robocopbackendssafy.duckdns.org/api/v1/map')
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