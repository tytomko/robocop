<template>
    <div>
      <div class="dashboard">
        <StatsMetricBox
          v-for="(metric, index) in metrics"
          :key="index"
          :title="metric.title"
          :count="metric.count"
          :class="metric.className"
        />
      </div>
  
      <!-- Filter Section -->
      <div class="filter-section">
        <label for="robot-select">로봇 선택:</label>
        <select id="robot-select" v-model="selectedRobot">
          <option value="all">모든 로봇</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
  
        <!-- Date Range Picker -->
        <label for="date-range">기간 설정:</label>
        <input type="date" v-model="startDate" /> ~ 
        <input type="date" v-model="endDate" />
      </div>
  
      <!-- Charts -->
      <StatsChart :robots="filteredRobots" :selectedRobot="selectedRobot" />
  
      <!-- Table -->
      <StatsTable :robots="filteredRobots" :selectedRobot="selectedRobot" />
    </div>
  </template>
  
  <script setup>
  import { ref, onMounted, watch, computed } from 'vue'
  import { useRobotsStore } from '@/stores/robots'
  import StatsMetricBox from '@/components/statistics/StatsMetricBox.vue'
  import StatsChart from '@/components/statistics/StatsChart.vue'
  import StatsTable from '@/components/statistics/StatsTable.vue'
  
  const robotsStore = useRobotsStore()
  const robots = ref([])
  const selectedRobot = ref('all')
  
  // Date range model
  const startDate = ref('')
  const endDate = ref('')
  
  // Metrics 배열 (나중에 updateMetrics()로 갱신)
  const metrics = ref([
    { title: '활동 중인 로봇', count: 0, className: 'active-robots' },
    { title: '충전 중인 로봇', count: 0, className: 'charging-robots' },
    { title: '고장 난 로봇', count: 0, className: 'broken-robots' },
    { title: '수리 중인 로봇', count: 0, className: 'fixing-robots' },
  ])
  
  // 선택한 로봇이 'all'이면 전체 로봇, 아니면 해당 로봇만 반환하는 computed 속성
  const filteredRobots = computed(() => {
    return selectedRobot.value === 'all'
      ? robots.value
      : robots.value.filter(robot => robot.id === selectedRobot.value)
  })
  
  // Metrics 업데이트 함수
  const updateMetrics = () => {
    const fr = filteredRobots.value
    console.log(fr)
    metrics.value = [
      { title: '활동 중인 로봇', count: fr.filter(r => r.status !== 'breakdown').length, className: 'active-robots' },
      { title: '충전 중인 로봇', count: fr.filter(r => r.status === 'charging').length, className: 'charging-robots' },
      { title: '고장 난 로봇', count: fr.filter(r => r.status === 'broken').length, className: 'broken-robots' },
      { title: '수리 중인 로봇', count: fr.filter(r => r.status === 'fixing').length, className: 'fixing-robots' },
    ]
  }
  
  // 로봇 목록을 불러오는 함수
  const loadRobots = async () => {
    await robotsStore.loadRobots()  // 비동기 로딩
    robots.value = robotsStore.registered_robots
    updateMetrics() // 데이터 로드 후 metrics 갱신
  }
  
  onMounted(async () => {
    await loadRobots()
  })
  
  // 로봇 목록이나 선택한 로봇이 바뀔 때마다 metrics 갱신
  watch([robots, selectedRobot], () => {
    updateMetrics()
  })
  </script>
  
  <style scoped>
  .dashboard {
    display: flex;
    gap: 15px;
    justify-content: center;
  }
  
  .filter-section {
    margin-bottom: 20px;
  }
  
  /* Left Border Colors */
  .active-robots {
    border-left: 4px solid #1e88e5;
  }
  
  .charging-robots {
    border-left: 4px solid #0c9d46;
  }
  
  .broken-robots {
    border-left: 4px solid #fbc02d;
  }
  
  .fixing-robots {
    border-left: 4px solid #f57c00;
  }
  </style>  