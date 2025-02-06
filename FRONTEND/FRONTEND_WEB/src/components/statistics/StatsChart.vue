<template>
    <div class="charts">
      <div class="chart-box">
        <h3>활동 시간</h3>
        <canvas ref="activityChartRef"></canvas>
      </div>
      <div class="chart-box">
        <h3>배터리</h3>
        <canvas ref="batteryChartRef"></canvas>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref, watch, onMounted } from 'vue'
  import Chart from 'chart.js/auto'
  
  const props = defineProps({
    robots: Array,
    selectedRobot: String
  })
  
  const activityChartRef = ref(null)
  const batteryChartRef = ref(null)
  let activityChartInstance = null
  let batteryChartInstance = null
  
  const initCharts = () => {
    if (!activityChartRef.value || !batteryChartRef.value) return
  
    // 기존 차트가 존재하면 파괴
    if (activityChartInstance) {
      activityChartInstance.destroy()
    }
    if (batteryChartInstance) {
      batteryChartInstance.destroy()
    }

    // 만약 데이터가 없다면 차트를 그리지 않음
  if (!props.robots || props.robots.length === 0) return
  
    const ctxActivity = activityChartRef.value.getContext('2d')
    const ctxBattery = batteryChartRef.value.getContext('2d')
  
    // 활동 시간 데이터 계산
    const activityData = props.robots.map(robot => {
      const startTime = new Date(robot.starttime)
      const currentTime = new Date()
      return (currentTime - startTime) / (1000 * 60 * 60) // 시간 단위로 변환
    })
  
    // 배터리 레벨 데이터
    const batteryData = props.robots.map(robot => robot.battery)
  
    // 새로운 차트 생성
    activityChartInstance = new Chart(ctxActivity, {
      type: 'bar',
      data: {
        labels: props.robots.map(robot => robot.name),
        datasets: [{
          label: '활동 시간 (시간)',
          data: activityData,
          backgroundColor: '#1e88e5',
          borderRadius: 5
        }]
      },
      options: {
        plugins: { legend: { display: false } },
        scales: { y: { beginAtZero: true } }
      }
    })
  
    batteryChartInstance = new Chart(ctxBattery, {
      type: 'bar',
      data: {
        labels: props.robots.map(robot => robot.name),
        datasets: [{
          label: '배터리 (%)',
          data: batteryData,
          backgroundColor: '#0c9d46',
          borderRadius: 5
        }]
      },
      options: {
        plugins: { legend: { display: false } },
        scales: { y: { beginAtZero: true, max: 100 } }
      }
    })
  }

// selectedRobot가 바뀔 때마다 차트를 다시 그리도록
watch(() => props.selectedRobot, () => {
  initCharts()
})
  
// 로봇 데이터가 변경될 때도 차트를 초기화하도록 watch 추가 (immediate 옵션 사용)
watch(() => props.robots, () => {
  initCharts()
}, { immediate: true })
  
  onMounted(() => {
    initCharts()
  })
  </script>
  
  <style scoped>
.charts {
  display: flex;
  gap: 20px;
  margin-bottom: 20px;
  justify-content: center;
}

.chart-box {
  width: 45%;
  padding: 15px;
  background: #fff;
  border-radius: 10px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  text-align: center;
  border: 1px solid #e0e0e0;
}

.chart-box h3 {
  font-size: 16px;
  margin-bottom: 10px;
  color: #333;
}
  </style>
  