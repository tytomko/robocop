<template>
  <div class="flex gap-5 mb-5 justify-center">
    <div class="w-1/2 p-4 bg-white rounded-lg shadow-md border border-gray-200 text-center">
      <h3 class="text-lg font-semibold text-gray-800 mb-3">활동 시간</h3>
      <canvas ref="activityChartRef"></canvas>
    </div>
    <div class="w-1/2 p-4 bg-white rounded-lg shadow-md border border-gray-200 text-center">
      <h3 class="text-lg font-semibold text-gray-800 mb-3">배터리</h3>
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

  if (activityChartInstance) {
    activityChartInstance.destroy()
  }
  if (batteryChartInstance) {
    batteryChartInstance.destroy()
  }

  if (!props.robots || props.robots.length === 0) return

  const ctxActivity = activityChartRef.value.getContext('2d')
  const ctxBattery = batteryChartRef.value.getContext('2d')

  const activityData = props.robots.map(robot => {
    const startTime = new Date(robot.startAt)
    const currentTime = new Date()
    return (currentTime - startTime) / (1000 * 60 * 60)
  })

  const batteryData = props.robots.map(robot => robot.battery)

  activityChartInstance = new Chart(ctxActivity, {
    type: 'bar',
    data: {
      labels: props.robots.map(robot => robot.nickname),
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
      labels: props.robots.map(robot => robot.nickname),
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

watch(() => props.selectedRobot, () => {
  initCharts()
})

watch(() => props.robots, () => {
  initCharts()
}, { immediate: true })

onMounted(() => {
  initCharts()
})
</script>
