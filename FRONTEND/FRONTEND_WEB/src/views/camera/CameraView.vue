<template>
  <div class="h-screen overflow-hidden bg-gray-100">
    <!-- 상단 컨트롤 섹션 -->
    <div class="p-4 bg-white shadow-sm">
      <div class="max-w-screen-xl mx-auto">
        <div class="flex justify-between items-start gap-4">
          <!-- 로봇 선택 (왼쪽) -->
          <div class="w-3/4">
            <div class="flex flex-col gap-2">
              <label class="font-medium text-gray-700">로봇 선택</label>
              <div class="w-full h-[120px] overflow-y-auto border border-gray-300 rounded bg-white text-sm">
                <div 
                  v-for="robot in robotsStore.registered_robots" 
                  :key="robot.seq"
                  @click="toggleRobotSelection(robot.seq)"
                  :class="[
                    'p-2 cursor-pointer hover:bg-gray-100 flex items-center',
                    selectedRobots.includes(robot.seq) ? 'bg-blue-100 hover:bg-blue-200' : ''
                  ]"
                >
                  <span>{{ robot.nickname || robot.manufactureName }}</span>
                  <svg
                    v-if="selectedRobots.includes(robot.seq)"
                    xmlns="http://www.w3.org/2000/svg"
                    class="h-5 w-5 text-blue-600"
                    viewBox="0 0 20 20"
                    fill="currentColor"
                  >
                    <path
                      fill-rule="evenodd"
                      d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z"
                      clip-rule="evenodd"
                    />
                  </svg>
                </div>
              </div>
            </div>
          </div>

          <!-- 카메라 방향 선택 (오른쪽) -->
          <div class="w-1/4">
            <div class="flex flex-col gap-2">
              <label class="font-medium text-gray-700">카메라 방향</label>
              <select
                v-model="selectedDirection"
                class="w-full p-2 border border-gray-300 rounded bg-white text-sm"
              >
                <option value="both">전/후방</option>
                <option value="front">전방만</option>
                <option value="rear">후방만</option>
              </select>
              <p class="text-sm text-gray-500">
                {{ selectedDirection === 'both' ? '1개' : '2개' }}의 로봇을 선택할 수 있습니다. 
                (현재 {{ selectedRobots.length }}개 선택됨)
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 카메라 화면 영역 -->
    <div class="flex-1 p-4">
      <div v-if="selectedRobots.length > 0" 
           :class="['video-container', gridClass]">
        <Cctv
          v-for="camera in cameras"
          :key="`${camera.robotSeq}-${camera.type}`"
          :robotSeq="camera.robotSeq"
          :cameraType="camera.type"
          class="video-item"
        />
      </div>
      <div v-else class="h-full flex items-center justify-center text-gray-500">
        로봇을 선택해주세요
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch, onBeforeUnmount } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/camera/Cctv.vue'

const robotsStore = useRobotsStore()
const selectedRobots = ref([])
const selectedDirection = ref('both')

// 선택된 로봇 수가 제한을 초과하는지 감시
watch([selectedRobots, selectedDirection], ([newRobots, newDirection]) => {
  const max = newDirection === 'both' ? 1 : 2
  if (newRobots.length > max) {
    // 최근 선택된 로봇들만 유지
    selectedRobots.value = newRobots.slice(-max)
    // localStorage 업데이트
    localStorage.setItem('selectedRobots', JSON.stringify(selectedRobots.value))
  }
})

const showFrontCamera = computed(() => 
  selectedDirection.value === 'both' || selectedDirection.value === 'front'
)

const showRearCamera = computed(() => 
  selectedDirection.value === 'both' || selectedDirection.value === 'rear'
)

const gridClass = computed(() => {
  const camerasPerRobot = selectedDirection.value === 'both' ? 2 : 1
  const totalCameras = selectedRobots.value.length * camerasPerRobot
  if (totalCameras === 1) return "single"
  if (totalCameras === 2) return "double"
  return "grid-2x1" // 새로운 그리드 클래스
})

const cameras = computed(() => {
  const result = []
  selectedRobots.value.forEach(robotSeq => {
    if (showFrontCamera.value) {
      result.push({ robotSeq, type: 'front' })
    }
    if (showRearCamera.value) {
      result.push({ robotSeq, type: 'rear' })
    }
  })
  return result
})

const toggleRobotSelection = (robotSeq) => {
  const maxAllowed = selectedDirection.value === 'both' ? 1 : 2
  
  if (selectedRobots.value.includes(robotSeq)) {
    // 이미 선택된 로봇이면 선택 해제
    selectedRobots.value = selectedRobots.value.filter(seq => seq !== robotSeq)
  } else {
    // 선택되지 않은 로봇인 경우
    if (selectedRobots.value.length >= maxAllowed) {
      // 최대 선택 개수에 도달한 경우, 가장 오래된 선택을 제거하고 새로운 선택 추가
      selectedRobots.value = [...selectedRobots.value.slice(1), robotSeq]
    } else {
      // 최대 선택 개수에 도달하지 않은 경우, 새로운 선택 추가
      selectedRobots.value.push(robotSeq)
    }
  }
  
  // localStorage 업데이트
  localStorage.setItem('selectedRobots', JSON.stringify(selectedRobots.value))
}

onMounted(() => {
  robotsStore.loadRobots()
  // 메인 컨텐츠의 스크롤 방지
  const mainContent = document.querySelector('.main-content')
  if (mainContent) {
    mainContent.style.overflow = 'hidden'
  }
  
  const savedRobots = localStorage.getItem('selectedRobots')
  if (savedRobots) {
    const parsedRobots = JSON.parse(savedRobots)
    selectedRobots.value = Array.isArray(parsedRobots)
      ? parsedRobots.filter(seq => robotsStore.registered_robots.some(robot => robot.seq === seq))
      : []
  }
  if (robotsStore.selectedRobot && !selectedRobots.value.includes(robotsStore.selectedRobot)) {
    selectedRobots.value.push(robotsStore.selectedRobot)
  }
})

onBeforeUnmount(() => {
  // 컴포넌트 언마운트 시 스크롤 복구
  const mainContent = document.querySelector('.main-content')
  if (mainContent) {
    mainContent.style.overflow = 'auto'
  }
})
</script>

<style scoped>
.video-container {
  display: grid;
  width: 100%;
  height: calc(90vh - 200px); /* 상단 컨트롤 영역이 줄어든 만큼 높이 조정 */
  gap: 1rem;
  background: black;
  padding: 1rem;
  border-radius: 0.5rem;
}

.video-item {
  width: 100%;
  height: 100%;
  background: #1a1a1a;
  border-radius: 0.5rem;
  overflow: hidden;
}

/* 그리드 레이아웃 */
.single {
  grid-template-columns: 1fr;
}

.double {
  grid-template-columns: repeat(2, 1fr);
}

.grid-2x1 {
  grid-template-columns: repeat(2, 1fr);
  grid-template-rows: 1fr;
}
</style>