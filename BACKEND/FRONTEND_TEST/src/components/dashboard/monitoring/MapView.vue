<template>
  <div class="map-view">
    <div class="map-header">
      <h3>맵 뷰</h3>
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
  toolbox: {
    feature: {
      restore: {},
      dataZoom: {
        yAxisIndex: 'none'
      }
    },
    right: 20,
    top: 20
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