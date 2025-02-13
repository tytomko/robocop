<template>
  <div class="relative h-full">
    <!-- 사이드바 컨테이너 (오른쪽 고정, 너비를 애니메이션으로 변경) -->
    <div
      class="absolute top-0 right-0 h-full bg-gray-100 border-l border-gray-200 transition-all duration-300 ease-in-out overflow-auto"
      :class="isCollapsed ? 'w-0' : 'w-[400px]'"
    >
      <!-- 실제 사이드바 내부 콘텐츠 -->
      <div class="flex flex-col gap-1 min-h-full">
        <div class="bg-white">
          <div class="flex items-center mb-2 h-[55px] p-4 border-b border-gray-200 bg-white">
            <h3 class="m-0 text-gray-800 font-bold text-base">
              실시간 로봇 현황
            </h3>
          </div>
          <div class="p-1 flex flex-col gap-1">
            <RobotMap />
          </div>
        </div>
      </div>
    </div>

    <!-- 토글 버튼 (사이드바 왼쪽에 배치) -->
    <button
      class="toggle-button"
      @click="toggleSidebar"
      :class="{ collapsed: isCollapsed }"
    >
      {{ isCollapsed ? '⏴' : '⏵' }}
    </button>
  </div>
</template>

<script setup>
import RobotMap from '@/components/map/RobotMap.vue';
import { useRobotsStore } from '@/stores/robots';
import { onMounted, nextTick, watch } from 'vue';

const props = defineProps({
  isCollapsed: {
    type: Boolean,
    default: false,
  },
});
const emit = defineEmits(['toggle-sidebar']);

const toggleSidebar = () => {
  emit('toggle-sidebar');
  nextTick(() => {
    const chart = document.querySelector('.chart');
    if (chart) {
      chart.dispatchEvent(new Event('resize'));
    }
  });
};

// 로봇 데이터 스토어 (예: Pinia)
const robotsStore = useRobotsStore();
onMounted(() => {
  robotsStore.loadRobots();
  const savedRobot = localStorage.getItem('selectedRobot');
  if (savedRobot) {
    robotsStore.selectedRobot = savedRobot;
  }
});

watch(() => props.isCollapsed, async (newVal) => {
  if (!newVal) { // 사이드바가 열릴 때
    await nextTick();
    const chart = document.querySelector('.chart');
    if (chart) {
      chart.dispatchEvent(new Event('resize'));
      console.log("사이드바 열림 → ECharts 리사이즈 실행");
    }
  }
});
</script>

<style scoped>
/* 토글 버튼의 위치/스타일 (원하는 대로 조정) */
.toggle-button {
  position: absolute;
  top: 50%;
  left: -20px;
  transform: translateY(-50%);
  background-color: #f8f9fa;
  border: 1px solid #bbb;
  border-radius: 10px;
  width: 40px;
  height: 40px;
  cursor: pointer;
  z-index: 10;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 16px;
  font-weight: bold;
  color: #555;
  transition: background-color 0.3s;
}

.toggle-button:hover {
  background-color: #e1e1e1;
}
</style>
