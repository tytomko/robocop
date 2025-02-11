<template>
  <div class="min-h-screen flex flex-col overflow-auto">
    <div class="p-5 flex-1">
      <div class="dashboard flex gap-4 justify-center">
        <div 
          v-for="(metric, index) in metrics" 
          :key="index"
          class="p-4 shadow-md rounded-lg bg-white relative flex flex-col items-start w-52 border-l-8"
          :class="metric.borderClass"
        >
          <p class="text-gray-700 text-sm">{{ metric.title }}</p>
          <p class="text-2xl font-bold">{{ metric.count }}대</p>
        </div>
      </div>

      <!-- Filter Section -->
      <div class="filter-section my-5 flex flex-col md:flex-row gap-4 items-center">
        <label for="robot-select" class="text-gray-700">로봇 선택:</label>
        <select id="robot-select" v-model="selectedRobot" class="p-2 border rounded-lg">
          <option value="all">모든 로봇</option>
          <option v-for="robot in robots" :key="robot.seq" :value="robot.seq">
            {{ robot.nickname }}
          </option>
        </select>

        <!-- Date Range Picker -->
        <label for="date-range" class="text-gray-700">기간 설정:</label>
        <input type="date" v-model="startDate" class="p-2 border rounded-lg" /> ~ 
        <input type="date" v-model="endDate" class="p-2 border rounded-lg" />
      </div>

      <!-- Charts -->
      <div class="overflow-auto">
        <StatsChart :robots="filteredRobots" :selectedRobot="selectedRobot" class="my-5" />
      </div>

      <!-- Table -->
      <div class="overflow-auto">
        <StatsTable :robots="filteredRobots" :selectedRobot="selectedRobot" class="my-5" />
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch, computed } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import StatsChart from '@/components/statistics/StatsChart.vue'
import StatsTable from '@/components/statistics/StatsTable.vue'

const robotsStore = useRobotsStore()
const robots = ref([])
const selectedRobot = ref('all')

// Date range model
const startDate = ref('')
const endDate = ref('')

const metrics = ref([
  { title: '활동 중인 로봇', count: 0, borderClass: 'border-blue-500' },
  { title: '충전 중인 로봇', count: 0, borderClass: 'border-green-500' },
  { title: '고장 난 로봇', count: 0, borderClass: 'border-yellow-500' },
  { title: '수리 중인 로봇', count: 0, borderClass: 'border-orange-500' },
])

const filteredRobots = computed(() => {
  return selectedRobot.value === 'all'
    ? robots.value
    : robots.value.filter(robot => robot.seq === selectedRobot.value)
})

const updateMetrics = () => {
  const fr = filteredRobots.value
  metrics.value = [
    { title: '활동 중인 로봇', count: fr.filter(r => r.isActive !== 'false').length, borderClass: 'border-blue-500' },
    { title: '충전 중인 로봇', count: fr.filter(r => r.status === 'charging').length, borderClass: 'border-green-500' },
    { title: '고장 난 로봇', count: fr.filter(r => r.status === 'error').length, borderClass: 'border-yellow-500' },
    { title: '휴식 중인 로봇', count: fr.filter(r => r.isActive === 'false').length, borderClass: 'border-orange-500' },
  ]
}

const loadRobots = async () => {
  await robotsStore.loadRobots()
  robots.value = robotsStore.registered_robots
  updateMetrics()
}

onMounted(async () => {
  await loadRobots()
})

watch([robots, selectedRobot], () => {
  updateMetrics()
})
</script>
