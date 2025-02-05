<template>
  <div class="robot-control-page" @keydown="handleKeyDown" @keyup="handleKeyUp">
    <!-- 로봇 선택 -->
    <div class="robot-selection">
      <label for="robot-select">로봇 선택:</label>
      <select id="robot-select" v-model="selectedRobotId">
        <option disabled value="">선택해주세요</option>
        <option 
          v-for="robot in robotsStore.registered_robots" 
          :key="robot.id" 
          :value="robot.id">
          {{ robot.nickname || robot.name }}
        </option>
      </select>
    </div>

    <!-- 제어 모드 토글 스위치 -->
    <div class="mode-toggle">
      <span class="mode-label">수동</span>
      <div class="toggle-switch">
        <input type="checkbox" id="toggle" v-model="isAutoMode" @change="toggleMode">
        <label for="toggle" class="switch">
          <span class="slider"></span>
        </label>
      </div>
      <span class="mode-label">자동</span>
    </div>

    <!-- 선택된 로봇이 없으면 안내 메시지 출력 -->
    <div v-if="!activeRobot">
      로봇을 먼저 선택해주세요.
    </div>

    <!-- 제어 영역 -->
    <div class="control-area" v-if="activeRobot">
      <!-- 자동 모드: 지도와 위치 정보 표시 -->
      <div v-if="mode === 'auto'">
        <RobotMap :robot="activeRobot" />
      </div>

      <!-- 수동 제어 모드: CCTV와 키보드 화살표 컨트롤 표시 -->
      <div v-else-if="mode === 'manual'">
        <Cctv :robot="activeRobot" />
        <div class="arrow-controls">
          <button :class="{ active: activeArrow === 'ArrowUp' }">↑</button>
          <div class="horizontal-controls">
            <button :class="{ active: activeArrow === 'ArrowLeft' }">←</button>
            <button :class="{ active: activeArrow === 'ArrowRight' }">→</button>
          </div>
          <button :class="{ active: activeArrow === 'ArrowDown' }">↓</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/dashboard/Cctv.vue'
import RobotMap from '@/components/dashboard/RobotMap.vue'

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
/* HTML, BODY 전체적으로 스크롤 가능하도록 설정 */
html, body {
  height: 100%;
  overflow-y: auto;
}

/* 기본적인 스타일 수정 */
.robot-control-page {
  max-width: 800px;
  margin: 0 auto;
  padding: 20px;
  min-height: 150vh; /* 화면보다 더 크게 만들어서 스크롤 가능 */
  overflow-y: auto; /* 항상 스크롤 가능하도록 설정 */
  display: flex;
  flex-direction: column;
}

.robot-selection {
  margin-bottom: 20px;
}

.mode-toggle {
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 20px;
}

.mode-label {
  margin: 0 10px;
  font-size: 16px;
  font-weight: bold;
}

/* 토글 스위치 */
.toggle-switch {
  position: relative;
  display: inline-block;
  width: 120px;
  height: 40px;
}

.toggle-switch input {
  opacity: 0;
  width: 0;
  height: 0;
}

.switch {
  position: absolute;
  cursor: pointer;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: #a52a2a;
  border-radius: 40px;
  transition: 0.4s;
  display: flex;
  align-items: center;
  padding: 4px;
}

.slider {
  height: 32px;
  width: 32px;
  background-color: white;
  border-radius: 50%;
  transition: 0.4s;
  position: relative;
}

input:checked + .switch {
  background-color: #8bc34a;
}

input:checked + .switch .slider {
  transform: translateX(80px);
}

/* 컨트롤 영역 */
.control-area {
  margin-top: 20px;
}

/* 화살표 컨트롤 스타일 */
.arrow-controls {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin-top: 20px;
}

.horizontal-controls {
  display: flex;
  justify-content: center;
  margin: 10px 0;
}

.arrow-controls button {
  width: 50px;
  height: 50px;
  font-size: 24px;
  margin: 5px;
  border: 1px solid #ccc;
  background: #f9f9f9;
  cursor: pointer;
  transition: all 0.2s ease;
}

/* 키보드 입력 시 버튼이 활성화되도록 변경 */
.arrow-controls button.active {
  font-weight: bold;
  background-color: #007bff;
  color: #fff;
  transform: scale(1.1);
  box-shadow: 0px 0px 10px rgba(0, 123, 255, 0.5);
}
</style>
