<template>
  <div class="robot-control-page" @keydown="handleKeyDown" @keyup="handleKeyUp">
    <div class="robot-selection-and-mode">
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
    </div>

    <div v-if="!activeRobot">
      로봇을 먼저 선택해주세요.
    </div>

    <div class="control-area" v-if="activeRobot">
      <div v-if="mode === 'auto'">
        <RobotMap :robot="activeRobot" />
      </div>

      <div v-else-if="mode === 'manual'" class="manual-mode">
        <div class="cctv-and-controls">
          <!-- CCTV 컴포넌트 크기 확대 -->
          <Cctv :robot="activeRobot" />

          <!-- 화살표 컨트롤 -->
          <div class="arrow-controls">
            <div class="vertical-controls">
              <button :class="{ active: activeArrow === 'ArrowUp' }">↑</button>
            </div>
            <div class="horizontal-controls">
              <button :class="{ active: activeArrow === 'ArrowLeft' }">←</button>
              <button :class="{ active: activeArrow === 'ArrowDown' }">↓</button>
              <button :class="{ active: activeArrow === 'ArrowRight' }">→</button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue'
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
.robot-control-page {
  max-width: 100%;
  margin: 0 auto;
  padding: 20px;
  min-height: 150vh;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
}

.robot-selection-and-mode {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.robot-selection {
  margin-bottom: 0;
}

.mode-toggle {
  display: flex;
  align-items: center;
}

.mode-label {
  margin: 0 10px;
  font-size: 18px;
  font-weight: bold;
}

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

.manual-mode {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
}

.cctv-and-controls {
  display: flex;
  flex-direction: row; /* 세로로 배치 */
  align-items: center; /* 중앙 정렬 */
  width: 100%; /* 카메라 화면을 더 크게 만들기 위해 전체 너비의 90%로 설정 */
  margin: 0 auto; /* 가운데 정렬 */
}

.cctv-screen {
  width: 100%; /* CCTV 화면이 화면 전체 너비를 차지하도록 설정 */
  height: 500px; /* 카메라 화면의 높이를 크게 설정 */
  background-color: black; /* 배경을 검정색으로 설정 */
  margin-bottom: 20px; /* 카메라 화면과 화살표 컨트롤 간의 간격 */
}

.arrow-controls {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  margin-left: 15px;
}

.horizontal-controls {
  display: flex;
  justify-content: center;
  margin-bottom: 10px;
}

.vertical-controls {
  display: flex;
  flex-direction: column;
  justify-content: center;
}

.arrow-controls button {
  width: 60px; /* 버튼 크기 키움 */
  height: 60px;
  font-size: 28px; /* 텍스트 크기 증가 */
  margin: 8px;
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
