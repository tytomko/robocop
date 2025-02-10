<template>
  <div class="w-full h-full relative">
    <!-- 확장 상태일 때만 내부 콘텐츠 렌더링 -->
    <template v-if="!isCollapsed">
      <div class="flex flex-col gap-1 bg-gray-100 flex-1">
        <div class="bg-white">
          <div class="flex items-center mb-2 p-4 border-b border-gray-200 bg-white">
            <h3 class="m-0 text-gray-800 font-bold text-base">실시간 로봇 현황</h3>
          </div>
          <div class="p-4 flex flex-col gap-5">
            <RobotMap />
          </div>
        </div>
      </div>
    </template>
    <!-- 토글 버튼은 항상 표시 -->
    <button class="toggle-button" @click="toggleSidebar" :class="{ 'collapsed': isCollapsed }">
      {{ isCollapsed ? '⏴' : '⏵' }}
    </button>
  </div>
</template>

<script setup>
import { defineProps, defineEmits, onMounted } from 'vue';
import RobotMap from '@/components/map/RobotMap.vue';
import { useRobotsStore } from '@/stores/robots';

const props = defineProps({
  isCollapsed: {
    type: Boolean,
    default: false,
  },
});
const emit = defineEmits(["toggle-sidebar"]);

const toggleSidebar = () => {
  emit("toggle-sidebar");
};

const robotsStore = useRobotsStore();
onMounted(() => {
  robotsStore.loadRobots();
  const savedRobot = localStorage.getItem("selectedRobot");
  if (savedRobot) {
    robotsStore.selectedRobot = savedRobot;
  }
});
</script>

<style scoped>
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
  transition: background-color 0.3s, transform 0.3s;
}

/* 버튼에 마우스를 올릴 때 */
.toggle-button:hover {
  background-color: #e1e1e1;
}

/* 사이드바가 확장된 경우에만 아이콘 변경 */
.toggle-button:hover::before {
  content: "⏵";
}

/* 사이드바가 축소된 경우 아이콘 유지 */
.toggle-button.collapsed:hover::before {
  content: "⏴";
}
</style>