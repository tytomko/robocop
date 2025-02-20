<template>
  <div class="flex flex-col bg-white">
    <div class="flex-1 relative p-1">
      <div class="relative w-full h-[450px] min-h-[20px] mx-auto" ref="containerRef">
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
        @remove-node="handleNodeRemove"
      />

      <div v-if="loading" class="absolute inset-0 flex flex-col items-center justify-center bg-white bg-opacity-90">
        <div class="w-10 h-10 border-4 border-gray-300 border-t-blue-500 rounded-full animate-spin"></div>
        <span class="mt-3 text-gray-700">맵 데이터를 불러오는 중...</span>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch, onUnmounted } from 'vue'
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

// Store 초기화
const robotsStore = useRobotsStore()
const robotCommandsStore = useRobotCommandsStore()
const emit = defineEmits(['selectedNodesChange'])

// Refs 정의
const containerRef = ref(null)
const chartRef = ref(null)
const loading = ref(true)
const mapData = ref({ nodes: [], links: [] })
const selectedNodes = ref([])
const imageWidth = ref(800)
const imageHeight = ref(500)
const robotPositions = ref(new Map()) // 여러 로봇의 위치를 저장하는 Map

// Props 정의
const props = defineProps({
  showSelectedNodes: {
    type: Boolean,
    default: true
  },
  robot: {
    type: Object,
    required: false,
    default: () => null
  },
  isMonitoringMode: {
    type: Boolean,
    default: false
  },
  showNodes: {
    type: Boolean,
    default: true
  }
})

// Robot colors for monitoring mode
const robotColors = {
  0: '#0000ff',
  1: '#ffff00',
  2: '#00ff00',
  3: '#ff00ff',
  4: '#ff0000'
}

// Computed 속성
const currentRobotSeq = computed(() => {
  if (props.robot) {
    return props.robot.seq
  }

  const selectedRobotSeq = robotsStore.selectedRobot
  if (selectedRobotSeq) {
    const selectedRobot = robotsStore.robots.find(
      robot => robot.seq === selectedRobotSeq
    )
    return selectedRobot?.seq
  }
  return null
})

const selectedNodesInfo = computed(() => {
  return selectedNodes.value.map(node => ({
    x: node.id[0].toFixed(2),
    y: node.id[1].toFixed(2)
  }))
})

// 차트 옵션 computed
const chartOption = computed(() => {
  const robotSeries = []
  
  // 모든 로봇 위치 표시
  robotPositions.value.forEach((position, robotSeq) => {
    // position이 존재하고 x, y값이 모두 있는 경우에만 처리
    if (position && position.x != null && position.y != null) {
      const isSelectedRobot = !props.isMonitoringMode && robotSeq === currentRobotSeq.value;
      
      robotSeries.push({
        type: 'scatter',
        data: [[position.x, position.y]],
        symbolSize: 15,
        itemStyle: {
          color: isSelectedRobot ? '#ff0000' : robotColors[robotSeq % Object.keys(robotColors).length]
        },
        symbol: 'circle',
        zlevel: 3
      });
    }
  });

  return {
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
      show: true,
      trigger: 'item',
      confine: true,
      enterable: false,
      formatter: function(params) {
        if (!params.data) return '';
        
        // 로봇 시리즈인 경우
        if (params.seriesIndex < robotSeries.length) {
          const robotSeq = Array.from(robotPositions.value.keys())[params.seriesIndex];
          const robot = robotsStore.robots.find(r => r.seq === robotSeq);
          return robot ? `로봇: ${robot.nickname || robot.manufactureName || robotSeq}` : '';
        }
        
        // 노드 시리즈인 경우 - monitoring mode가 아닐 때만 좌표 표시
        if (!props.isMonitoringMode && params.componentSubType === 'scatter' && Array.isArray(params.data)) {
          try {
            return `좌표: (${Number(params.data[0]).toFixed(2)}, ${Number(params.data[1]).toFixed(2)})`;
          } catch (e) {
            return '';
          }
        }
        
        return '';
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
      ...robotSeries,
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
          ) ? 20 : 8
        },
        itemStyle: {
          color: (params) => {
            const node = mapData.value.nodes[params.dataIndex]
            return selectedNodes.value.some(selected => 
              selected.id[0] === node.id[0] && selected.id[1] === node.id[1]
            ) ? '#ff4081' : '#007bff'
          }
        },
        label: {
          show: true,
          formatter: (params) => {
            const node = mapData.value.nodes[params.dataIndex]
            const index = selectedNodes.value.findIndex(selected => 
              selected.id[0] === node.id[0] && selected.id[1] === node.id[1]
            )
            return index !== -1 ? (index + 1).toString() : ''
          },
          color: '#fff',
          fontSize: 12,
          fontWeight: 'bold',
          position: 'inside'
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
  }
})

function updateChartSeries() {
  // Early return if required refs are not available
  if (!chartRef.value || !mapData.value) {
    console.warn('Chart reference or map data not available');
    return;
  }

  const seriesData = [];

  // Add lines series (paths between nodes)
  if (mapData.value.links && Array.isArray(mapData.value.links)) {
    seriesData.push({
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
    });
  }

  // Add nodes series (waypoints)
  if (mapData.value.nodes && Array.isArray(mapData.value.nodes)) {
    seriesData.push({
      type: 'scatter',
      data: mapData.value.nodes.map(node => [node.id[0], node.id[1]]),
      symbolSize: (value) => {
        return selectedNodes.value?.some(sel => 
          sel.id[0] === value[0] && sel.id[1] === value[1]
        ) ? 20 : 8;
      },
      itemStyle: {
        color: (params) => {
          const node = mapData.value.nodes[params.dataIndex];
          return selectedNodes.value?.some(sel =>
            sel.id[0] === node.id[0] && sel.id[1] === node.id[1]
          ) ? '#ff4081' : '#007bff';
        }
      },
      label: {
        show: true,
        formatter: (params) => {
          const node = mapData.value.nodes[params.dataIndex];
          const index = selectedNodes.value?.findIndex(sel => 
            sel.id[0] === node.id[0] && sel.id[1] === node.id[1]
          );
          return index !== -1 ? (index + 1).toString() : '';
        },
        color: '#fff',
        fontSize: 12,
        fontWeight: 'bold',
        position: 'inside'
      },
      emphasis: {
        scale: 1.5,
        itemStyle: {
          shadowBlur: 10,
          shadowColor: 'rgba(0, 0, 0, 0.3)'
        }
      },
      zlevel: 2
    });
  }

  // Add robot position series based on mode
  robotPositions.value?.forEach((position, robotSeq) => {
    if (position && 
        typeof position.x === 'number' && 
        typeof position.y === 'number' &&
        !isNaN(position.x) && 
        !isNaN(position.y)) {
      const isSelectedRobot = !props.isMonitoringMode && robotSeq === currentRobotSeq.value;
      
      seriesData.push({
        type: 'scatter',
        data: [[position.x, position.y]],
        symbolSize: 15,
        itemStyle: {
          color: isSelectedRobot ? '#ff0000' : robotColors[robotSeq % Object.keys(robotColors).length]
        },
        symbol: 'circle',
        zlevel: 3,
        tooltip: {
          formatter: () => {
            const robot = robotsStore.robots.find(r => r.seq === robotSeq);
            return `로봇: ${robot?.nickname || robot?.manufactureName || robotSeq}`;
          }
        }
      });
    }
  });

  // Safely update chart options
  try {
    chartRef.value.setOption({
      series: seriesData
    }, { 
      lazyUpdate: true,
      silent: true // Suppress unnecessary warnings
    });
  } catch (error) {
    console.error('Failed to update chart series:', error);
  }
}

// SSE 설정
function setupSSE() {
  // 기존 연결들 정리
  robotPositions.value.clear() // 기존 위치 정보도 초기화
  eventSources.forEach(source => source.close())
  eventSources.clear()
  
  // seq가 1과 2인 로봇에 대해서만 SSE 설정
  const newEventSources = new Map()
  
  robotsStore.robots
    .filter(robot => robot.seq === 1 || robot.seq === 2)
    .forEach(robot => {
      console.log(`Setting up SSE for robot ${robot.seq}`) // 디버깅용
      const url = `https://robocopbackendssafy.duckdns.org/api/v1/robots/sse/${robot.seq}/down-utm`
      const eventSource = new EventSource(url)
      
      let lastUpdate = 0
      const updateInterval = 500

      eventSource.onmessage = (event) => {
        const now = Date.now()
        if (now - lastUpdate < updateInterval) return

        const data = JSON.parse(event.data)
        if (data && data.position && 
              typeof data.position.x === 'number' && 
              typeof data.position.y === 'number') {
            robotPositions.value.set(robot.seq, {
              x: data.position.x,
              y: data.position.y
            })}
        lastUpdate = now
      }

      eventSource.onerror = (error) => {
        console.error(`SSE 연결 에러 (로봇 ${robot.seq}):`, error)
        eventSource.close()
        robotPositions.value.delete(robot.seq) // 에러 시 위치 정보도 삭제
      }

      newEventSources.set(robot.seq, eventSource)
    })

  return newEventSources
}

// 로봇 제어 함수들
async function handleNavigate() {
  try {
    await robotCommandsStore.navigateCommand(selectedNodes.value, currentRobotSeq.value)
    // 명령 전송 후 선택된 노드 초기화
    selectedNodes.value = []
    updateChartSeries()
    emit('selectedNodesChange', selectedNodes.value)
  } catch (error) {
    console.error('Navigation command failed:', error)
  }
}

async function handlePatrol() {
  try {
    await robotCommandsStore.patrolCommand(selectedNodes.value, currentRobotSeq.value)
    // 명령 전송 후 선택된 노드 초기화
    selectedNodes.value = []
    updateChartSeries()
    emit('selectedNodesChange', selectedNodes.value)
  } catch (error) {
    console.error('Patrol command failed:', error)
  }
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

// 노드 클릭 핸들러
function handleNodeClick(params) {
  // 모니터링 모드에서는 노드 클릭 비활성화
  if (props.isMonitoringMode) return
  
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

    updateChartSeries()
    emit('selectedNodesChange', selectedNodes.value)
  }
}

// 노드 제거 핸들러
const handleNodeRemove = ({ node }) => {
  const index = selectedNodes.value.findIndex(n => 
    n.id[0].toFixed(2) === node.x && n.id[1].toFixed(2) === node.y
  )
  
  if (index !== -1) {
    selectedNodes.value = selectedNodes.value.filter((_, i) => i !== index)
    updateChartSeries()
    emit('selectedNodesChange', selectedNodes.value)
  }
}

// 맵 데이터 fetch
async function fetchMapData() {
  try {
    loading.value = true
    const response = await axios.get('https://robocopbackendssafy.duckdns.org/api/v1/map')
    mapData.value = { nodes: response.data.nodes, links: response.data.links }
    updateChartSeries()
  } catch (error) {
    console.error('맵 데이터 로딩 실패:', error)
  } finally {
    loading.value = false
  }
}

// Watchers
let eventSources = new Map()

// robotsStore.robots가 변경될 때 SSE 재설정
watch(() => robotsStore.robots, (newRobots) => {
  console.log('Robots changed:', newRobots.map(r => r.seq))
  if (eventSources.size > 0) {
    console.log('Closing existing SSE connections')
    eventSources.forEach(source => source.close())
    eventSources.clear()
  }
  eventSources = setupSSE()
}, { deep: true })

// currentRobotSeq 변경 시에는 선택된 로봇만 업데이트
watch(() => currentRobotSeq.value, () => {
  updateChartSeries()
})

watch([() => robotPositions.value], () => {
  if (chartRef.value && mapData.value) {
    updateChartSeries()
  }
}, { deep: true })

watch(selectedNodes, () => {
  updateChartSeries()
})

watch(() => props.robot, (newRobot) => {
  if (newRobot) {
    console.log('Robot changed in RobotMap:', newRobot)
    selectedNodes.value = []
    updateChartSeries()
  }
}, { deep: true })

// Lifecycle hooks
onMounted(() => {
  const resizeObserver = new ResizeObserver((entries) => {
    for (const entry of entries) {
      const { width, height } = entry.contentRect
      imageWidth.value = width * 0.9
      imageHeight.value = height * 0.95
      
      if (chartRef.value) {
        chartRef.value.resize()
      }
    }
  })

  if (containerRef.value) {
    resizeObserver.observe(containerRef.value)
  }

  fetchMapData()
  eventSources = setupSSE()
})

onUnmounted(() => {
  console.log('Cleaning up SSE connections...') // 디버깅용
  eventSources.forEach(source => {
    source.close()
    console.log('Closed SSE connection') // 디버깅용
  })
  eventSources.clear()
  robotPositions.value.clear() // 위치 정보도 정리
})

// 외부로 노출할 메서드
defineExpose({
  handleNavigate,
  handlePatrol,
  resetSelection,
  handleTempStop,
  handleResume
})
</script>