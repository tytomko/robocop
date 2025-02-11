<!-- ListSidebarSection.vue -->
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

      <!-- 로봇 리스트 헤더 (위에 여백을 주고 싶다면 mt-6 등 추가 가능) -->
      <div class="p-4 border-b border-gray-700 mt-6">
        <span class="text-sm font-medium uppercase tracking-wider text-gray-300">
          로봇 리스트
        </span>
      </div>

      <!-- 로봇 목록 -->
      <div class="overflow-auto flex-1">
        <ul>
          <li
            v-for="robot in robots"
            :key="robot.seq"
            class="flex items-center px-4 py-2 border-b border-gray-700 hover:bg-gray-800 cursor-pointer"
          >
            <!-- 활성/비활성 아이콘 -->
            <span
              class="inline-block w-3 h-3 rounded-full mr-2"
              :class="robot.isActive ? 'bg-green-500' : 'bg-gray-500'"
            ></span>
            <!-- 접혀있으면 로봇 이름 숨김 -->
            <span v-if="!isCollapsed">
              {{ robot.nickname || robot.name }}
            </span>
          </li>
        </ul>
      </div>
    </div>

    <!-- 토글 버튼: 사이드바 너비에 따라 왼쪽에서 슬라이드되도록 transition-all 적용 -->
    <button
      class="absolute top-1/2 transform -translate-y-1/2 bg-gray-200 text-black px-2 py-1 rounded-r hover:bg-gray-300 z-10 
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
</script>
