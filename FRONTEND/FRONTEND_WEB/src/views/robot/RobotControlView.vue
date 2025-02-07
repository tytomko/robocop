<template>
  <div class="robot-control-page" @keydown="handleKeyDown" @keyup="handleKeyUp">
    <div class="robot-selection-and-mode flex justify-between items-center mb-5">
      <div class="robot-selection">
        <label for="robot-select" class="font-semibold">로봇 선택:</label>
        <select id="robot-select" v-model="selectedRobotId" class="mt-2 p-2 border border-gray-300 rounded-md w-48">
          <option disabled value="">선택해주세요</option>
          <option 
            v-for="robot in robotsStore.registered_robots" 
            :key="robot.id" 
            :value="robot.id">
            {{ robot.nickname || robot.name }}
          </option>
        </select>
      </div>

      <div class="mode-toggle flex items-center">
        <span class="mode-label text-lg font-semibold mr-4">수동</span>
        <div class="toggle-switch relative inline-block w-24 h-12">
          <!-- 여기에 id를 지정하여 label과 연결되도록 했습니다. -->
          <input 
            type="checkbox" 
            id="toggle" 
            v-model="isAutoMode" 
            @change="toggleMode"
            class="opacity-0 w-0 h-0"
          />
          <!-- label의 for 속성이 input의 id와 연결되어야 합니다. -->
          <label for="toggle" class="switch relative inline-block w-full h-full cursor-pointer rounded-full bg-gray-400 transition-all">
            <span class="slider absolute top-0 left-0 w-10 h-10 bg-white rounded-full transition-transform"></span>
          </label>
        </div>
        <span class="mode-label text-lg font-semibold ml-4">자동</span>
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
          <Cctv :robot="activeRobot" class="w-full h-96 bg-black" />

          <div class="arrow-controls flex flex-col items-center ml-5">
            <div class="vertical-controls flex flex-col justify-center items-center mb-5">
              <button :class="{ 'active': activeArrow === 'ArrowUp' }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 transition transform hover:scale-110">↑</button>
            </div>
            <div class="horizontal-controls flex justify-center">
              <button :class="{ 'active': activeArrow === 'ArrowLeft' }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 mx-2 transition transform hover:scale-110">←</button>
              <button :class="{ 'active': activeArrow === 'ArrowDown' }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 mx-2 transition transform hover:scale-110">↓</button>
              <button :class="{ 'active': activeArrow === 'ArrowRight' }" class="w-16 h-16 text-3xl border border-gray-300 bg-gray-100 hover:bg-blue-500 rounded-full mb-2 mx-2 transition transform hover:scale-110">→</button>
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

// 선택된 로봇 ID (select의 v-model)
const selectedRobotId = ref('')

// activeRobot computed: 등록된 로봇이 존재할 때만 선택
const activeRobot = computed(() => {
  return robotsStore.registered_robots.find(robot => String(robot.id) === String(selectedRobotId.value)) || null
})

// 제어 모드 (기본값 'auto' : 자동 주행 모드)
const mode = ref('auto')

// 키보드 입력으로 눌린 화살표 상태
const activeArrow = ref(null)

// 키보드 이벤트 감지
function handleKeyDown(event) {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
    activeArrow.value = event.key
  }
}

function handleKeyUp(event) {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
    activeArrow.value = null
  }
}

// 모드 토글 스위치
const isAutoMode = ref(true)
function toggleMode() {
  mode.value = isAutoMode.value ? 'auto' : 'manual'
}

// 컴포넌트가 마운트될 때 로봇 데이터를 로드합니다.
onMounted(() => {
  robotsStore.loadRobots()
})
</script>

<style scoped>
/* 추가적인 CSS를 사용하고 싶으면 아래에 넣을 수 있습니다. */
.robot-control-page {
  min-height: 100vh;
}
.arrow-controls button.active {
  font-weight: bold;
  background-color: #007bff;
  color: white;
  transform: scale(1.1);
  box-shadow: 0px 0px 10px rgba(0, 123, 255, 0.5);
}

/* 토글 스위치 활성화 상태 배경 색 */
.switch {
  background-color: #d1d5db; /* 비활성화 상태 배경 색 */
  transition: background-color 0.4s ease-in-out;
}

/* 체크된 상태에서 배경색과 슬라이더의 이동 */
.switch input:checked + .slider {
  transform: translateX(12px); /* 슬라이더 이동 */
  background-color: #4caf50; /* 활성화 상태 배경 색 */
}

/* 슬라이더 애니메이션과 배경색 */
.slider {
  background-color: white; /* 기본 슬라이더 배경 색 */
  transition: transform 0.4s ease, background-color 0.4s ease; /* 부드러운 애니메이션 */
}
</style>
