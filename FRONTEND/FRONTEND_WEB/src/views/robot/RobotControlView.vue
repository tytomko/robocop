<template>
  <div class="robot-control-page" @keydown="handleKeyDown" @keyup="handleKeyUp">
    <div class="robot-selection-and-mode flex justify-between items-center mb-5">
      <div class="robot-selection">
        <label for="robot-select" class="font-semibold">로봇 선택:</label>
        <select id="robot-select" v-model="selectedRobotSeq" class="custom-select">
          <option disabled value="">선택해주세요</option>
          <option v-for="robot in robotsStore.registered_robots" :key="robot.seq" :value="robot.seq">
            {{ robot.nickname || robot.name }}
          </option>
        </select>
      </div>

      <div class="mode-toggle flex items-center space-x-3">
        <span class="mode-label">수동</span>
        <label class="toggle-switch">
          <input type="checkbox" v-model="isAutoMode" @change="toggleMode" />
          <span class="slider"></span>
        </label>
        <span class="mode-label">자동</span>
      </div>
    </div>

    <div v-if="!activeRobot" class="text-center text-gray-500">로봇을 먼저 선택해주세요.</div>

    <div class="control-area mt-5" v-if="activeRobot">
      <div v-if="mode === 'auto'">
        <!-- (1) 버튼 컴포넌트 -->
        <ControlButtons :selectedNodes="selectedNodes" @navigate="handleNavigate" @patrol="handlePatrol" @reset="resetSelection" @tempStop="handleTempStop"/>

        <!-- 선택된 노드 표시 -->
        <SelectedNodes :selectedNodes="selectedNodes" />

        <!-- (2) RobotMap -->
        <RobotMap ref="robotMap" :robot="activeRobot" @selectedNodesChange="onSelectedNodesChange" />
      </div>

      <div v-else-if="mode === 'manual'" class="manual-mode flex justify-between items-start">
        <div class="cctv-and-controls flex flex-row items-center w-full">
          <Cctv :robot="activeRobot" class="cctv-screen" />

          <div class="arrow-controls flex flex-col items-center ml-5">
            <button :class="{ active: activeArrows.has('ArrowUp') }" class="control-btn arrow">↑</button>
            <div class="horizontal-controls flex justify-center mt-2">
              <button :class="{ active: activeArrows.has('ArrowLeft') }" class="control-btn arrow">←</button>
              <button :class="{ active: activeArrows.has('ArrowDown') }" class="control-btn arrow">↓</button>
              <button :class="{ active: activeArrows.has('ArrowRight') }" class="control-btn arrow">→</button>
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
import ControlButtons from '@/components/map/ControlButtons.vue'
import SelectedNodes from '@/components/map/SelectedNodes.vue'

const robotsStore = useRobotsStore()
const selectedRobotSeq = ref('')

// 선택된 노드 상태 추가
const selectedNodes = ref([])

const activeRobot = computed(() => {
  return robotsStore.registered_robots.find(robot => String(robot.seq) === String(selectedRobotSeq.value)) || null
})

const mode = ref('auto')

// 🚨 선택된 노드 변경 이벤트 (정상적으로 실행되는지 확인)
function onSelectedNodesChange(newNodes) {
  console.log('[RobotControlView] selectedNodes changed:', newNodes) // 디버깅 로그 추가
  selectedNodes.value = [...newNodes] // 🚨 Vue의 반응형 상태를 유지하면서 값 변경
}

// 🚨 selectedNodes가 변경될 때마다 watch를 통해 로그 확인
watch(selectedNodes, (newVal) => {
  console.log('Watch detected selectedNodes change:', newVal)
})

// RobotMap의 메서드를 직접 쓰기 위해 ref로 잡기
const robotMap = ref(null)

// 버튼 클릭 시 -> RobotMap 내부 함수 호출
function handleNavigate() {
  robotMap.value?.handleNavigate?.()
}
function handlePatrol() {
  robotMap.value?.handlePatrol?.()
}
function resetSelection() {
  selectedNodes.value = []
}
function handleTempStop() {
  robotMap.value?.handleTempStop?.()
}

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
  robotsStore.loadRobots()
  // 만약 store에 selectedRobot 이 있으면 그것을 기본값으로 설정
  if (robotsStore.selectedRobot) {
    selectedRobotSeq.value = String(robotsStore.selectedRobot)
  }
})
</script>

<style scoped>
/* 📌 모달 및 페이지 기본 설정 */
.robot-control-page {
  min-height: 100vh;
  background-color: #f9fafb;
  padding: 20px;
}

/* 📌 드롭다운 스타일 */
.custom-select {
  margin-top: 5px;
  padding: 10px;
  border: 2px solid #ddd;
  border-radius: 8px;
  width: 200px;
  font-size: 16px;
  background-color: white;
  transition: all 0.3s;
}
.custom-select:hover {
  border-color: #007bff;
}
.custom-select:focus {
  outline: none;
  border-color: #0056b3;
  box-shadow: 0px 0px 8px rgba(0, 91, 255, 0.4);
}

/* 📌 토글 스위치 */
.toggle-switch {
  position: relative;
  width: 60px;
  height: 30px;
  display: inline-block;
}
.toggle-switch input {
  display: none;
}
.toggle-switch .slider {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: #d1d5db;
  border-radius: 30px;
  transition: 0.4s;
}
.toggle-switch input:checked + .slider {
  background-color: #4caf50;
}
.toggle-switch .slider:before {
  content: "";
  position: absolute;
  height: 26px;
  width: 26px;
  background: white;
  border-radius: 50%;
  top: 2px;
  left: 2px;
  transition: 0.4s;
}
.toggle-switch input:checked + .slider:before {
  transform: translateX(30px);
}

/* 📌 버튼 기본 스타일 */
.control-btn {
  width: 60px;
  height: 60px;
  border: 3px solid #007bff;
  background-color: white;
  color: #007bff;
  font-size: 24px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s;
  box-shadow: 0px 5px 10px rgba(0, 123, 255, 0.2);
}
.control-btn:hover {
  background-color: #007bff;
  color: white;
  transform: scale(1.1);
}
.control-btn:active {
  transform: scale(0.95);
  box-shadow: none;
}

/* 📌 활성화된 버튼 */
.control-btn.active {
  background-color: #0056b3;
  color: white;
  box-shadow: 0px 5px 15px rgba(0, 91, 255, 0.5);
}

/* 📌 CCTV 스타일 */
.cctv-screen {
  width: 100%;
  height: 400px;
  background: black;
  border-radius: 8px;
  box-shadow: 0px 5px 15px rgba(0, 0, 0, 0.2);
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  font-size: 20px;
}
</style>