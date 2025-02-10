<template>
  <div class="p-5 h-[90vh] overflow-hidden flex flex-col">
    <!-- 실시간 모니터링 섹션 -->
    <div class="flex flex-col gap-5 h-full">
      <div class="p-2 border-b border-gray-200 mt-2">
        <!-- 로봇 선택 -->
        <select
          v-model="selectedRobots"
          class="w-full p-2 border border-gray-300 rounded bg-white text-sm appearance-none bg-no-repeat bg-right pr-8"
          @change="handleRobotSelection"
          multiple
        >
          <option v-for="robot in robotsStore.registered_robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
      </div>

      <!-- 선택된 로봇 카메라들 (화면 내에서 크기 자동 조정) -->
      <div v-if="selectedRobots.length > 0" :class="gridClass" class="video-container">
        <Cctv
          v-for="robotId in selectedRobots"
          :key="robotId"
          :cameraName="'카메라 ' + robotId"
          :cameraStatus="'연결 대기 중'"
          :streamInfo="streamInfoMap[robotId]"
          class="video-item"
        />
        <div v-if="selectedRobots.length === 3" class="video-item empty"></div>
      </div>
      <div v-else class="p-5 text-center text-gray-500">로봇을 선택해주세요</div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/camera/Cctv.vue'

// robotsStore 활용하기
const robotsStore = useRobotsStore()

// 선택된 로봇들 (초기값 빈 배열)
const selectedRobots = ref([])
// 각 로봇의 스트림 정보
const streamInfoMap = ref({})

// 선택한 로봇 개수에 따라 배치 스타일 변경
const gridClass = computed(() => {
  const count = selectedRobots.value.length
  if (count === 1) return "single"
  if (count === 2) return "double"
  if (count === 3) return "triple"
  return "quad" // 4개일 때
})

// 로봇 선택 후 상태 변경
const handleRobotSelection = () => {
  selectedRobots.value.forEach(robotId => {
    streamInfoMap.value[robotId] = robotsStore.getStreamInfo(robotId)
  })
}

// 로봇 데이터 불러오기
onMounted(() => {
  robotsStore.loadRobots()
  const savedRobots = localStorage.getItem('selectedRobots')
  if (savedRobots) {
    const parsedRobots = JSON.parse(savedRobots)
    selectedRobots.value = Array.isArray(parsedRobots) ? parsedRobots : []
  }
})

// selectedRobots가 변경될 때마다 로컬 스토리지에 저장
watch(selectedRobots, (newRobots) => {
  localStorage.setItem('selectedRobots', JSON.stringify(newRobots))
})
</script>

<style scoped>
/* 전체 컨테이너 */
.video-container {
  display: flex;
  width: 100%;
  height: 100%;
  justify-content: flex-start; /* 왼쪽 정렬 */
  align-items: flex-start; /* 상단 정렬 */
  flex-wrap: wrap;
  background: black; /* 항상 검은색 배경 유지 */
}

/* CCTV 화면 크기 자동 조절 */
.video-item {
  background: black;
  border-radius: 10px;
  overflow: hidden;
}

/* 빈 공간을 위한 스타일 */
.video-item.empty {
  width: 50%;
  height: 50%;
  background: black;
}

/* 1개 선택 시 전체 화면 */
.single .video-item {
  width: 100%;
  height: 100%;
}

/* 2개 선택 시 가로 분할 */
.double .video-item {
  width: 50%;
  height: 100%;
}

/* 3개 선택 시: 왼쪽 정렬 + 2x2 형태로 배치 */
.triple .video-item:nth-child(1),
.triple .video-item:nth-child(2) {
  width: 50%;
  height: 50%;
}

.triple .video-item:nth-child(3) {
  width: 50%;
  height: 50%;
}

.triple .video-item.empty {
  width: 50%;
  height: 50%;
  background: black;
}

/* 4개 선택 시 2×2 */
.quad .video-item {
  width: 50%;
  height: 50%;
}
</style>
