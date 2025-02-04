<template>
  <div class="robot-control-page">
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
    <div v-if="!selectedRobot">
      로봇을 먼저 선택해주세요.
    </div>

    <!-- 제어 영역 -->
    <div class="control-area" v-if="selectedRobot">
      <!-- 자동 모드: 지도와 위치 정보 표시 -->
      <div v-if="mode === 'auto'">
        <RobotMap :robot="selectedRobot" />
      </div>

      <!-- 수동 제어 모드: CCTV와 화살표 컨트롤 표시 -->
      <div v-else-if="mode === 'manual'">
        <Cctv :robot="selectedRobot" />
        <div class="arrow-controls">
          <button
            :class="{ active: activeArrow === 'up' }"
            @mousedown="pressArrow('up')"
            @mouseup="releaseArrow"
            @mouseleave="releaseArrow"
          >
            ↑
          </button>
          <div class="horizontal-controls">
            <button
              :class="{ active: activeArrow === 'left' }"
              @mousedown="pressArrow('left')"
              @mouseup="releaseArrow"
              @mouseleave="releaseArrow"
            >
              ←
            </button>
            <button
              :class="{ active: activeArrow === 'right' }"
              @mousedown="pressArrow('right')"
              @mouseup="releaseArrow"
              @mouseleave="releaseArrow"
            >
              →
            </button>
          </div>
          <button
            :class="{ active: activeArrow === 'down' }"
            @mousedown="pressArrow('down')"
            @mouseup="releaseArrow"
            @mouseleave="releaseArrow"
          >
            ↓
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/dashboard/Cctv.vue'
import RobotMap from '@/components/dashboard/RobotMap.vue'

// Pinia store에서 로봇 데이터를 불러옵니다.
const robotsStore = useRobotsStore()

// 선택된 로봇 ID (select box의 v-model)
const selectedRobotId = ref('')

// 선택된 로봇 객체 (ID에 따라 computed로 찾아냄)
const selectedRobot = computed(() =>
  (robotsStore.registered_robots.value || []).find(
    robot => String(robot.id) === selectedRobotId.value
  )
)

// 제어 모드 (기본값 'auto' : 자동 주행 모드)
const mode = ref('auto')

// 수동 제어 모드에서 눌린 화살표 상태
const activeArrow = ref(null)

// 화살표 버튼을 누르면 activeArrow 업데이트
function pressArrow(direction) {
  activeArrow.value = direction
  // 여기에 로봇 제어 관련 API 호출 등 추가 로직 구현 가능
}

// 버튼에서 손을 떼거나 마우스가 벗어나면 activeArrow 초기화
function releaseArrow() {
  activeArrow.value = null
}

// 모드 토글 스위치 관련 (예시로 isAutoMode를 활용)
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
  max-width: 800px;
  margin: 0 auto;
  padding: 20px;
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

.arrow-controls button.active {
  font-weight: bold;
  background-color: #007bff;
  color: #fff;
}
</style>
