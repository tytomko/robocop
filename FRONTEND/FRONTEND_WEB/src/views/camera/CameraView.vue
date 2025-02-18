<template>
  <div class="h-full flex flex-col bg-gray-100">
    <!-- 상단 컨트롤 섹션 -->
    <div class="p-4 bg-white shadow-sm">
      <div class="max-w-screen-xl mx-auto space-y-4">
        <!-- 로봇 선택 -->
        <div class="flex flex-col sm:flex-row gap-4 items-start sm:items-center">
          <label class="min-w-[100px] font-medium text-gray-700">로봇 선택</label>
          <div class="flex-1 space-y-2">
            <div class="w-full max-h-48 overflow-y-auto border border-gray-300 rounded bg-white text-sm">
              <div 
                v-for="robot in robotsStore.registered_robots" 
                :key="robot.seq"
                @dblclick="toggleRobotSelection(robot.seq)"
                :class="[
                  'p-2 cursor-pointer hover:bg-gray-100',
                  selectedRobots.includes(robot.seq) ? 'bg-blue-100 hover:bg-blue-200' : ''
                ]"
              >
                {{ robot.nickname || robot.manufactureName }}
              </div>
            </div>
            <p class="text-sm text-gray-500">
              {{ selectedDirection === 'both' ? '최대 2개' : '최대 4개' }}의 로봇을 선택할 수 있습니다. 
              (현재 {{ selectedRobots.length }}개 선택됨)
            </p>
          </div>
        </div>

        <!-- 카메라 방향 선택 -->
        <div class="flex flex-col sm:flex-row gap-4 items-start sm:items-center">
          <label class="min-w-[100px] font-medium text-gray-700">카메라 방향</label>
          <select
            v-model="selectedDirection"
            class="w-full sm:w-48 p-2 border border-gray-300 rounded bg-white text-sm"
          >
            <option value="both">전/후방</option>
            <option value="front">전방만</option>
            <option value="rear">후방만</option>
          </select>
        </div>
      </div>
    </div>

    <!-- 카메라 화면 영역 -->
    <div class="flex-1 p-4 min-h-0">
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
import { ref, computed, onMounted, watch } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/camera/Cctv.vue'

const robotsStore = useRobotsStore()
const selectedRobots = ref([])
const selectedDirection = ref('both')

// 최대 선택 가능한 로봇 수 계산
const maxRobots = computed(() => 
  selectedDirection.value === 'both' ? 2 : 4
)

// 선택된 로봇 수가 제한을 초과하는지 감시
watch([selectedRobots, selectedDirection], ([newRobots, newDirection]) => {
  const max = newDirection === 'both' ? 2 : 4
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
  if (totalCameras === 3) return "triple"
  return "quad"
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
  const maxAllowed = selectedDirection.value === 'both' ? 2 : 4
  
  if (selectedRobots.value.includes(robotSeq)) {
    // 이미 선택된 로봇이면 선택 해제
    selectedRobots.value = selectedRobots.value.filter(seq => seq !== robotSeq)
  } else if (selectedRobots.value.length < maxAllowed) {
    // 선택되지 않은 로봇이고 최대 선택 개수를 넘지 않으면 선택 추가
    selectedRobots.value.push(robotSeq)
  }
  
  // localStorage 업데이트
  localStorage.setItem('selectedRobots', JSON.stringify(selectedRobots.value))
}

onMounted(() => {
  robotsStore.loadRobots()
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
</script>

<style scoped>
/* 전체 컨테이너 */
.video-container {
  display: grid;
  width: 100%;
  height: calc(100vh - 200px); /* 상단 컨트롤 영역 높이를 제외한 나머지 */
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

/* 빈 공간을 위한 스타일 */
.video-item.empty {
  width: 50%;
  height: 50%;
  background: black;
}

/* 그리드 레이아웃 */
.single {
  grid-template-columns: 1fr;
}

.double {
  grid-template-columns: repeat(2, 1fr);
}

.triple {
  grid-template-columns: repeat(2, 1fr);
  grid-template-rows: repeat(2, 1fr);
}

.triple .video-item:last-child {
  grid-column: 1 / 2;
}

.quad {
  grid-template-columns: repeat(2, 1fr);
  grid-template-rows: repeat(2, 1fr);
}
</style>
