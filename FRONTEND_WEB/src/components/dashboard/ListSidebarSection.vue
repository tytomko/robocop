<template>
  <div class="relative h-full">
    <div
      class="bg-gray-900 text-white h-full overflow-hidden transition-all duration-300 ease-in-out flex flex-col"
      :class="isCollapsed ? 'w-0' : 'w-[240px]'"
    >
      <!-- 상단 로고 -->
      <div class="px-4 py-3 border-b border-gray-700 h-[55px] bg-gray-800 flex items-center flex-shrink-0">
        <h1 class="text-xl font-bold tracking-wide">ROBOCOP</h1>
      </div>

      <!-- 로봇 리스트 헤더 -->
      <div class="p-4 border-b border-gray-700 bg-gray-800/50 flex-shrink-0">
        <span class="text-sm font-medium uppercase tracking-wider text-gray-300">
          로봇 리스트
        </span>
        <div v-if="selectedRobotItem" class="mt-2 px-2 py-1 bg-gray-800 rounded-md">
          <span class="text-xs text-gray-300">선택된 로봇:</span>
          <span class="text-sm text-white ml-1 font-medium">{{ selectedRobotItem.nickname }}</span>
        </div>
      </div>

      <!-- 로봇 목록 -->
      <div class="flex-1 min-h-0"> <!-- min-h-0 추가 -->
        <div class="h-full overflow-y-auto py-2">
          <ul class="space-y-1 px-2">
            <li
              v-for="robot in robots"
              :key="robot.seq"
              class="rounded-lg transition-all duration-200"
              :class="[
                robotsStore.selectedRobot === robot.seq 
                  ? 'bg-gray-700 shadow-md' 
                  : 'hover:bg-gray-800/70'
              ]"
            >
              <button
                class="w-full px-4 py-3 flex items-center gap-3"
                @click="toggleRobotSelection(robot.seq)"
              >
                <!-- 상태 표시 -->
                <span
                  class="relative flex h-3 w-3"
                  :class="isActive(robot) ? 'text-green-500' : 'text-gray-500'"
                >
                  <span class="absolute inline-flex h-full w-full rounded-full opacity-75"
                        :class="isActive(robot) ? 'animate-ping bg-green-500' : 'bg-gray-500'"></span>
                  <span class="relative inline-flex rounded-full h-3 w-3"
                        :class="isActive(robot) ? 'bg-green-500' : 'bg-gray-500'"></span>
                </span>

                <!-- 로봇 이름 -->
                <span v-if="!isCollapsed" 
                      class="flex-1 text-sm font-medium"
                      :class="robotsStore.selectedRobot === robot.seq ? 'text-white' : 'text-gray-300'">
                  {{ robot.nickname || robot.name }}
                </span>

                <!-- 선택 표시 -->
                <span v-if="robotsStore.selectedRobot === robot.seq" 
                      class="text-green-400">
                  <svg xmlns="http://www.w3.org/2000/svg" 
                      class="h-5 w-5" 
                      viewBox="0 0 20 20" 
                      fill="currentColor">
                    <path fill-rule="evenodd" 
                          d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" 
                          clip-rule="evenodd" />
                  </svg>
                </span>
              </button>
            </li>
          </ul>
        </div>
      </div>
    </div>

    <!-- 토글 버튼 -->
    <button
      class="absolute top-1/2 -translate-y-1/2 bg-gray-800 hover:bg-gray-700 
             text-white w-6 h-20 rounded-r-md transition-all duration-300 ease-in-out
             flex items-center justify-center shadow-lg"
      :class="isCollapsed ? 'left-0' : 'left-[240px]'"
      @click="$emit('toggle-left-sidebar')"
    >
      <span class="text-sm">
        {{ isCollapsed ? '▶' : '◀' }}
      </span>
    </button>
  </div>
</template>

<script setup>
import { computed, onMounted, watch } from 'vue'
import { useRobotsStore } from '@/stores/robots'

const props = defineProps({
  isCollapsed: {
    type: Boolean,
    default: false,
  },
})
defineEmits(['toggle-left-sidebar'])

const robotsStore = useRobotsStore()
const robots = computed(() => {
  return robotsStore.displayRobots.slice().sort((a, b) => {
    const isActiveA = isActive(a) ? 1 : 0;
    const isActiveB = isActive(b) ? 1 : 0;
    return isActiveB - isActiveA; // 활성화된 로봇을 위로 정렬
  });
});

const isActive = (robot) => {
  return /^robot_\d+$/.test(robot.manufactureName) && 
         (robot.isActive === true || robot.IsActive === true);
}

onMounted(() => {
  // 새로고침 후에도 선택한 로봇 유지
  const savedRobot = localStorage.getItem('selectedRobot')
  if (savedRobot) {
    robotsStore.selectedRobot = parseInt(savedRobot, 10) // 숫자로 변환
  }

  robotsStore.loadRobots()
})

// 선택된 로봇 객체
const selectedRobotItem = computed(() => {
  return robots.value.find(r => r.seq === robotsStore.selectedRobot) || null
})

// robots 데이터가 변경될 때 selectedRobotItem 자동 업데이트
watch(robots, () => {
  if (!selectedRobotItem.value && robotsStore.selectedRobot) {
    robotsStore.selectedRobot = 0 // 선택한 로봇이 리스트에 없으면 초기화
  }
})

function toggleRobotSelection(seq) {
  // 혹시 seq가 문자열이면 parseInt 처리 (백엔드가 문자열 "4" 보내는 경우)
  const parsedSeq = typeof seq === 'string' ? parseInt(seq, 10) : seq

  // 이미 선택된 로봇이면 해제(0)
  if (robotsStore.selectedRobot === parsedSeq) {
    robotsStore.selectedRobot = 0
  } else {
    // 새 로봇 선택
    robotsStore.selectedRobot = parsedSeq
  }

  // 로컬 스토리지에 반영
  robotsStore.handleRobotSelection()
}
</script>
