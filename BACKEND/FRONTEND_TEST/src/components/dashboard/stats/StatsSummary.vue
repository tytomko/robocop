<template>
  <div class="stats-summary-container">
    <div class="page-header">
      <h2>통계 요약</h2>
      <div class="header-actions">
        <select v-model="selectedRobot" class="robot-select">
          <option value="">전체 로봇</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
        <select v-model="selectedPeriod" class="period-select">
          <option value="day">일간</option>
          <option value="week">주간</option>
          <option value="month">월간</option>
        </select>
      </div>
    </div>

    <div class="stats-grid">
      <!-- 운영 시간 -->
      <div class="stats-card">
        <div class="stats-icon">
          <i class="fas fa-clock"></i>
        </div>
        <div class="stats-content">
          <h3>운영 시간</h3>
          <div class="stats-value">{{ operatingHours }}시간</div>
          <div class="stats-trend" :class="{ 'up': operatingTrend > 0, 'down': operatingTrend < 0 }">
            {{ Math.abs(operatingTrend) }}% {{ operatingTrend > 0 ? '증가' : '감소' }}
          </div>
        </div>
      </div>

      <!-- 이동 거리 -->
      <div class="stats-card">
        <div class="stats-icon">
          <i class="fas fa-route"></i>
        </div>
        <div class="stats-content">
          <h3>이동 거리</h3>
          <div class="stats-value">{{ distance }}km</div>
          <div class="stats-trend" :class="{ 'up': distanceTrend > 0, 'down': distanceTrend < 0 }">
            {{ Math.abs(distanceTrend) }}% {{ distanceTrend > 0 ? '증가' : '감소' }}
          </div>
        </div>
      </div>

      <!-- 배터리 사용량 -->
      <div class="stats-card">
        <div class="stats-icon">
          <i class="fas fa-battery-half"></i>
        </div>
        <div class="stats-content">
          <h3>배터리 사용량</h3>
          <div class="stats-value">{{ batteryUsage }}%</div>
          <div class="stats-trend" :class="{ 'up': batteryTrend > 0, 'down': batteryTrend < 0 }">
            {{ Math.abs(batteryTrend) }}% {{ batteryTrend > 0 ? '증가' : '감소' }}
          </div>
        </div>
      </div>

      <!-- 특이사항 발생 -->
      <div class="stats-card">
        <div class="stats-icon">
          <i class="fas fa-exclamation-triangle"></i>
        </div>
        <div class="stats-content">
          <h3>특이사항 발생</h3>
          <div class="stats-value">{{ incidents }}건</div>
          <div class="stats-trend" :class="{ 'up': incidentsTrend > 0, 'down': incidentsTrend < 0 }">
            {{ Math.abs(incidentsTrend) }}% {{ incidentsTrend > 0 ? '증가' : '감소' }}
          </div>
        </div>
      </div>
    </div>

    <div class="charts-container">
      <!-- 시간별 운영 현황 -->
      <div class="chart-card">
        <h3>시간별 운영 현황</h3>
        <canvas ref="operationChart"></canvas>
      </div>

      <!-- 로봇별 운영 시간 -->
      <div class="chart-card">
        <h3>로봇별 운영 시간</h3>
        <canvas ref="robotChart"></canvas>
      </div>
    </div>

    <div v-if="loading" class="loading-overlay">
      <div class="loading-spinner"></div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import Chart from 'chart.js/auto'
import { webSocketService } from '@/services/websocket'

// 상태 변수
const selectedRobot = ref('')
const selectedPeriod = ref('day')
const loading = ref(false)
const operationChart = ref(null)
const robotChart = ref(null)
let operationChartInstance = null
let robotChartInstance = null

// 임시 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' }
])

// 통계 데이터
const operatingHours = ref(0)
const distance = ref(0)
const batteryUsage = ref(0)
const incidents = ref(0)
const operatingTrend = ref(5)
const distanceTrend = ref(-2)
const batteryTrend = ref(3)
const incidentsTrend = ref(-10)

// 차트 초기화
const initCharts = () => {
  if (operationChartInstance) {
    operationChartInstance.destroy()
  }
  if (robotChartInstance) {
    robotChartInstance.destroy()
  }

  // 시간별 운영 현황 차트
  const operationCtx = operationChart.value.getContext('2d')
  operationChartInstance = new Chart(operationCtx, {
    type: 'line',
    data: {
      labels: ['00:00', '03:00', '06:00', '09:00', '12:00', '15:00', '18:00', '21:00'],
      datasets: [{
        label: '운영 시간',
        data: [4, 6, 8, 7, 6, 8, 9, 7],
        borderColor: '#4CAF50',
        tension: 0.4,
        fill: false
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        legend: {
          display: false
        }
      },
      scales: {
        y: {
          beginAtZero: true
        }
      }
    }
  })

  // 로봇별 운영 시간 차트
  const robotCtx = robotChart.value.getContext('2d')
  robotChartInstance = new Chart(robotCtx, {
    type: 'bar',
    data: {
      labels: ['Robot 1', 'Robot 2', 'Robot 3', 'Robot 4'],
      datasets: [{
        label: '운영 시간',
        data: [12, 19, 15, 17],
        backgroundColor: [
          '#4CAF50',
          '#2196F3',
          '#FFC107',
          '#9C27B0'
        ]
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        legend: {
          display: false
        }
      },
      scales: {
        y: {
          beginAtZero: true
        }
      }
    }
  })
}

// 데이터 로드
const loadData = async () => {
  loading.value = true
  try {
    // 실제 API 호출로 대체 필요
    await new Promise(resolve => setTimeout(resolve, 1000))
    
    // 임시 데이터 설정
    operatingHours.value = Math.floor(Math.random() * 100)
    distance.value = Math.floor(Math.random() * 1000) / 10
    batteryUsage.value = Math.floor(Math.random() * 100)
    incidents.value = Math.floor(Math.random() * 20)
    
    initCharts()
  } catch (error) {
    console.error('데이터 로드 중 오류:', error)
  } finally {
    loading.value = false
  }
}

// 웹소켓 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/ws')
    
    webSocketService.subscribe('stats/update', (data) => {
      if (!data) return
      // 실시간 데이터 업데이트 처리
      updateStats(data)
    })
  } catch (error) {
    console.error('웹소켓 연결 실패:', error)
  }
}

// 실시간 데이터 업데이트
const updateStats = (data) => {
  // 데이터 업데이트 로직 구현
  console.log('실시간 데이터 업데이트:', data)
}

// 필터 변경 감지
watch([selectedRobot, selectedPeriod], () => {
  loadData()
})

onMounted(async () => {
  await loadData()
  await setupWebSocket()
})
</script>

<style scoped>
.stats-summary-container {
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

.robot-select,
.period-select {
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  min-width: 150px;
}

.stats-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 20px;
}

.stats-card {
  background: white;
  border-radius: 8px;
  padding: 20px;
  display: flex;
  align-items: center;
  gap: 20px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.stats-icon {
  width: 50px;
  height: 50px;
  background: #f5f5f5;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 24px;
  color: #666;
}

.stats-content {
  flex: 1;
}

.stats-content h3 {
  margin: 0;
  font-size: 14px;
  color: #666;
}

.stats-value {
  font-size: 24px;
  font-weight: bold;
  margin: 5px 0;
}

.stats-trend {
  font-size: 12px;
  color: #666;
}

.stats-trend.up {
  color: #4CAF50;
}

.stats-trend.down {
  color: #f44336;
}

.charts-container {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
  gap: 20px;
  flex: 1;
}

.chart-card {
  background: white;
  border-radius: 8px;
  padding: 20px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  height: 300px;
}

.chart-card h3 {
  margin: 0 0 20px 0;
  font-size: 16px;
  color: #333;
}

.loading-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(255, 255, 255, 0.8);
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