<template>
  <div class="robot-control-page" @keydown="handleKeyDown" @keyup="handleKeyUp">
    <div class="robot-selection-and-mode flex justify-between items-center mb-5">
      <div class="robot-selection">
        <label for="robot-select" class="font-semibold">로봇 선택:</label>
        <select id="robot-select" v-model="selectedRobotSeq" class="mt-2 p-2 border border-gray-300 rounded-md w-48">
          <option disabled value="">선택해주세요</option>
          <option 
            v-for="robot in robotsStore.registered_robots" 
            :key="robot.seq" 
            :value="robot.seq">
            {{ robot.nickname || robot.name }}
          </option>
        </select>
      </div>

      <div class="mode-toggle flex items-center space-x-3">
        <span class="mode-label text-lg font-semibold">수동</span>
        <div class="toggle-switch relative inline-block w-24 h-12 flex-shrink-0">
          <input 
            type="checkbox" 
            id="toggle" 
            v-model="isAutoMode" 
            @change="toggleMode"
            class="opacity-0 w-0 h-0"
          />
          <label for="toggle" class="switch relative inline-block w-full h-full cursor-pointer rounded-full transition-all flex items-center">
            <span class="slider absolute bg-white rounded-full"></span>
          </label>
        </div>
        <span class="mode-label text-lg font-semibold">자동</span>
      </div>
    </div>

    <div v-if="!activeRobot" class="text-center text-gray-500">
      로봇을 먼저 선택해주세요.
    </div>

    <div class="control-area mt-5" v-if="activeRobot">
      <div v-if="mode === 'auto'">
        <RobotMap :robot="activeRobot" />
      </div>

      <div v-else-if="mode === 'manual'" class="manual-mode flex justify-between items-start">
        <div class="cctv-and-controls flex flex-row items-center w-full">
          <Cctv :robot="activeRobot" class="w-full h-[490px] bg-black" />

          <div class="arrow-controls flex flex-col items-center ml-5">
            <div class="vertical-controls flex flex-col justify-center items-center mb-5">
              <button :class="{ 'active': activeArrows.has('ArrowUp') }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 transition transform hover:scale-110">↑</button>
            </div>
            <div class="horizontal-controls flex justify-center">
              <button :class="{ 'active': activeArrows.has('ArrowLeft') }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 mx-2 transition transform hover:scale-110">←</button>
              <button :class="{ 'active': activeArrows.has('ArrowDown') }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 mx-2 transition transform hover:scale-110">↓</button>
              <button :class="{ 'active': activeArrows.has('ArrowRight') }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 mx-2 transition transform hover:scale-110">→</button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/camera/Cctv.vue'
import RobotMap from '@/components/map/RobotMap.vue'

const robotsStore = useRobotsStore()
const selectedRobotSeq = ref('')

const activeRobot = computed(() => {
  return robotsStore.registered_robots.find(robot => String(robot.seq) === String(selectedRobotSeq.value)) || null
})

const mode = ref('auto')

const activeArrows = ref(new Set()) // 여러 개의 방향키 저장

function handleKeyDown(event) {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
    activeArrows.value.add(event.key)
  }
}

function handleKeyUp(event) {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
    activeArrows.value.delete(event.key)
  }
}

const isAutoMode = ref(true)
function toggleMode() {
  mode.value = isAutoMode.value ? 'auto' : 'manual'
}

onMounted(() => {
  robotsStore.loadRobots();
  // 만약 store에 selectedRobot 이 있으면 그것을 기본값으로 설정
  if (robotsStore.selectedRobot) {
    selectedRobotSeq.value = String(robotsStore.selectedRobot)
  }
})

import { watch } from 'vue';

watch(mode, (newMode) => {
  if (newMode === 'auto') {
    nextTick(() => {
      const chart = document.querySelector('.chart');
      if (chart) {
        chart.dispatchEvent(new Event('resize'));
      }
    });
  }
});

</script>

<style scoped>
.robot-control-page {
  min-height: 100vh;
}

.mode-toggle {
  display: flex;
  align-items: center;
  justify-content: center;
}

.toggle-switch {
  display: flex;
  align-items: center;
  justify-content: center;
}

/* 활성화된 상태 (초록색) */
input:checked + .switch {
  background-color: #4caf50;
}

/* 체크 상태일 때 슬라이더 이동 */
input:checked + .switch .slider {
  transform: translate(2.4rem, -50%); /* 오른쪽 끝까지 이동 */
}

/* 토글 버튼 기본 스타일 */
.switch {
  display: flex;
  align-items: center;
  justify-content: flex-start;
  background-color: #d1d5db;
  width: 5rem;  /* 80px */
  height: 2.5rem; /* 40px */
  border-radius: 9999px;
  transition: background-color 0.3s ease-in-out;
  position: relative;
}

/* 슬라이더 스타일 */
.slider {
  width: 2rem; /* 32px */
  height: 2rem; /* 32px */
  position: absolute;
  top: 50%;
  left: 0.3rem; /* 초기 위치 */
  transform: translateY(-50%);
  transition: transform 0.3s ease-in-out;
}

.arrow-controls button.active {
  font-weight: bold;
  background-color: #007bff;
  color: white;
  transform: scale(1.1);
  box-shadow: 0px 0px 10px rgba(0, 123, 255, 0.5);
}

</style>
