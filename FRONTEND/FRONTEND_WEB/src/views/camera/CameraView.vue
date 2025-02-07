<template>
  <div class="cctv-view">
    <!-- 실시간 모니터링 섹션 -->
    <div class="video-container">
      <div class="robot-selector">
        <!-- 로봇 선택 -->
        <select v-model="selectedRobots" class="robot-select" @change="handleRobotSelection" multiple>
          <option value="">로봇 선택</option>
          <option v-for="robot in robotsStore.registered_robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
      </div>

      <!-- 선택된 로봇 카메라들 -->
      <div v-if="selectedRobots.length > 0" class="camera-sections">
        <Cctv
          v-for="robotId in selectedRobots"
          :key="robotId"
          :cameraName="'카메라 ' + robotId"
          :cameraStatus="'연결 대기 중'"
          :streamInfo="streamInfoMap[robotId]"
        />
      </div>
      <div v-else class="no-robot-selected">로봇을 선택해주세요</div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/camera/Cctv.vue'

// robotsStore 활용하기
const robotsStore = useRobotsStore()

// 선택된 로봇들
const selectedRobots = ref([])
// 각 로봇의 스트림 정보
const streamInfoMap = ref({})

// 로봇 선택 후 상태 변경
const handleRobotSelection = () => {
  // 선택된 로봇들에 대해 스트림 정보 등을 갱신
  selectedRobots.value.forEach(robotId => {
    streamInfoMap.value[robotId] = robotsStore.getStreamInfo(robotId)
  })
}

// 로봇 데이터 불러오기
onMounted(() => {
  robotsStore.loadRobots() // 로봇 데이터 불러오기
  const savedRobots = localStorage.getItem('selectedRobots')
  if (savedRobots) {
    selectedRobots.value = JSON.parse(savedRobots)
  }
})

// selectedRobots가 변경될 때마다 로컬 스토리지에 저장
watch(selectedRobots, (newRobots) => {
  localStorage.setItem('selectedRobots', JSON.stringify(newRobots))
})
</script>
  
<style scoped>
.cctv-view {
  padding: 20px;
  height: 90vh; /* 화면의 전체 높이를 사용하도록 설정 */
  overflow-y: auto; /* 세로 스크롤 활성화 */
}

.video-container {
  display: flex;
  flex-direction: column;
  gap: 20px;
  height: auto; /* 내용에 맞게 높이를 자동으로 조절 */
  min-height: 100%; /* 최소 높이를 100%로 설정하여 스크롤 가능하게 유지 */
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

</style>
