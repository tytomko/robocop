<template>
  <div class="relative h-full">
    <!-- 사이드바 -->
    <div
      class="bg-gray-900 text-white h-full overflow-hidden transition-all duration-300 ease-in-out"
      :class="isCollapsed ? 'w-0' : 'w-[240px]'"
    >
      <!-- 상단 로고 -->
      <div class="p-3 border-b border-gray-700">
        <h1 class="text-xl font-bold">ROBOCOP</h1>
      </div>

      <!-- 로봇 리스트 헤더 -->
      <div class="p-4 border-b border-gray-700 mt-6">
        <span class="text-sm font-medium uppercase tracking-wider text-gray-300">
          로봇 리스트
        </span>
        <!-- 선택된 로봇 정보 표시 (있을 때만) -->
        <div v-if="selectedRobotItem" class="text-xs text-gray-200 mt-2">
          선택된 로봇: {{ selectedRobotItem.nickname }}
        </div>
      </div>

      <!-- 로봇 목록 -->
      <div class="overflow-auto flex-1">
        <ul>
          <li
            v-for="robot in robots"
            :key="robot.seq"
            class="flex items-center px-4 py-2 border-b border-gray-700 hover:bg-gray-800 cursor-pointer"
            @click="toggleRobotSelection(robot.seq)"
          >
            <!-- 체크 아이콘 -->
            <span
              v-if="robotsStore.selectedRobot.value === robot.seq"
              class="text-green-500 mr-2"
            >
              <i class="fas fa-check"></i>
            </span>

            <!-- 로봇 활성 표시 -->
            <span
              class="inline-block w-3 h-3 rounded-full mr-2"
              :class="robot.isActive ? 'bg-green-500' : 'bg-gray-500'"
            ></span>

            <span v-if="!isCollapsed">
              {{ robot.nickname || robot.name }}
            </span>
          </li>
        </ul>
      </div>
    </div>

    <!-- 토글 버튼 -->
    <button
      class="absolute top-1/2 transform -translate-y-1/2 bg-gray-300 text-black px-2 py-1 rounded-r hover:bg-gray-300 z-10 
             transition-all duration-300 ease-in-out"
      :class="isCollapsed ? 'left-0' : 'left-[240px]'"
      @click="$emit('toggle-left-sidebar')"
    >
      <span v-if="isCollapsed">▶</span>
      <span v-else>◀</span>
    </button>
  </div>
</template>

<script setup>
import { computed, onMounted } from 'vue'
import { useRobotsStore } from '@/stores/robots'

const props = defineProps({
  isCollapsed: {
    type: Boolean,
    default: false,
  },
})
defineEmits(['toggle-left-sidebar'])

const robotsStore = useRobotsStore()

onMounted(() => {
  robotsStore.loadRobots()
})

const robots = computed(() => robotsStore.registered_robots)

// 선택된 로봇 객체
const selectedRobotItem = computed(() => {
  return robots.value.find(r => r.seq === robotsStore.selectedRobot.value) || null
})

function toggleRobotSelection(seq) {
  // 혹시 seq가 문자열이면 parseInt 처리 (백엔드가 문자열 "4" 보내는 경우)
  const parsedSeq = typeof seq === 'string' ? parseInt(seq, 10) : seq

  // 이미 선택된 로봇이면 해제(0)
  if (robotsStore.selectedRobot.value === parsedSeq) {
    robotsStore.selectedRobot.value = 0
  } else {
    // 새 로봇 선택
    robotsStore.selectedRobot.value = parsedSeq
  }

  // 로컬 스토리지에 반영
  robotsStore.handleRobotSelection()
}
</script>
