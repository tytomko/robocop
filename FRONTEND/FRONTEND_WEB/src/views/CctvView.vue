<template>
    <div class="cctv-view">
      <!-- 실시간 모니터링 섹션 -->
      <div class="video-container">
        <div class="robot-selector">
          <!-- 로봇 선택 -->
          <select v-model="selectedRobot" class="robot-select" @change="handleRobotSelection">
            <option value="">로봇 선택</option>
            <option v-for="robot in registeredRobots" :key="robot.id" :value="robot.id">
              {{ robot.name }}
            </option>
          </select>
        </div>
  
        <!-- 선택된 로봇 카메라 -->
        <div v-if="selectedRobot" class="camera-sections">
          <CameraView cameraName="프론트캠" :cameraStatus="'연결 대기 중'" />
          <CameraView cameraName="리어캠" :cameraStatus="'연결 대기 중'" />
        </div>
        <div v-else class="no-robot-selected">로봇을 선택해주세요</div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref, onMounted, watch } from 'vue'
  import { useRobotsStore } from '@/stores/robots'
  import CameraView from '@/components/dashboard/CameraView.vue'
  
  // robotsStore 활용하기
  const robotsStore = useRobotsStore()
  
  // 로봇 관련 상태
  const selectedRobot = ref('')
  const registeredRobots = ref([])
  
  // 로봇 선택 후 상태 변경
  const handleRobotSelection = () => {
    if (selectedRobot.value) {
      localStorage.setItem('selectedRobot', selectedRobot.value)
    }
  }
  
  // 로봇 데이터 불러오기
  onMounted(() => {
    robotsStore.loadRobots() // 로봇 데이터 불러오기
    registeredRobots.value = robotsStore.registered_robots
    const savedRobot = localStorage.getItem('selectedRobot')
    if (savedRobot) {
      selectedRobot.value = savedRobot
    }
  })
  
  // 선택된 로봇 저장
  watch(selectedRobot, (newRobotId) => {
    if (newRobotId) {
      localStorage.setItem('selectedRobot', newRobotId)
    }
  })
  </script>
  
  <style scoped>
  .cctv-view {
    padding: 20px;
  }
  
  .video-container {
    display: flex;
    flex-direction: column;
    gap: 20px;
  }
  
  .robot-selector {
    padding: 8px 20px;
    margin-top: 8px;
    border-bottom: 1px solid #eee;
  }
  
  .robot-select {
    width: 100%;
    padding: 8px;
    border: 1px solid #ddd;
    border-radius: 4px;
    background: white;
    font-size: 14px;
  }
  
  .no-robot-selected {
    padding: 20px;
    text-align: center;
    color: #666;
  }
  
  /* 드롭다운 스타일 개선 */
  .robot-select {
    appearance: none;
    background-image: url("data:image/svg+xml;charset=UTF-8,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3e%3cpolyline points='6 9 12 15 18 9'%3e%3c/polyline%3e%3c/svg%3e");
    background-repeat: no-repeat;
    background-position: right 8px center;
    background-size: 16px;
    padding-right: 32px;
  }
  
  .robot-select:focus {
    outline: none;
    border-color: #007bff;
  }
  </style>
  