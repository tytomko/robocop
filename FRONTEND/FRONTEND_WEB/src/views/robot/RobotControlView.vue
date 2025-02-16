<template>
  <div 
      class="min-h-screen bg-gray-50 p-5 outline-none" 
      tabindex="0"
      ref="controlArea"
      @keydown="handleKeyDown"
      @keyup="handleKeyUp"
    >    
    <div class="flex justify-between items-center mb-5">
      <div class="space-y-2">
        <label for="robot-select" class="font-semibold block">로봇 선택:</label>
        <select 
          id="robot-select" 
          v-model="selectedRobotSeq" 
          class="w-52 p-2.5 border-2 border-gray-300 rounded-lg bg-white focus:border-blue-500 focus:ring-2 focus:ring-blue-200 transition-all duration-300"
        >
          <option disabled value="">선택해주세요</option>
          <option 
            v-for="robot in robotsStore.registered_robots" 
            :key="robot.seq" 
            :value="robot.seq"
          >
            {{ robot.nickname || robot.manufactureName }}
          </option>
        </select>
      </div>

      <!-- 수정된 토글 스위치 부분 -->
      <div class="flex items-center space-x-3">
        <span class="text-gray-700">수동</span>
        <label class="relative inline-block w-14 h-7">
          <input 
            type="checkbox" 
            v-model="isAutoMode" 
            @change="toggleMode"
            class="hidden"
          />
          <div 
            class="w-14 h-7 bg-gray-300 rounded-full cursor-pointer transition-colors duration-300"
            :class="{ 'bg-green-500': isAutoMode }"
          >
            <div 
              class="absolute left-1 top-1 w-5 h-5 bg-white rounded-full transition-transform duration-300"
              :class="{ 'translate-x-7': isAutoMode }"
            ></div>
          </div>
        </label>
        <span class="text-gray-700">자동</span>
      </div>
    </div>

    <div v-if="!activeRobot" class="text-center text-gray-500">
      로봇을 먼저 선택해주세요.
    </div>

    <div class="mt-5" v-if="activeRobot">
      <div v-if="mode === 'auto'">
        <ControlButtons 
          :selectedNodes="selectedNodes" 
          @navigate="handleNavigate" 
          @patrol="handlePatrol" 
          @reset="resetSelection" 
          @tempStop="handleTempStop"
        />

        <RobotMap 
          v-if="activeRobot"
          :key="mapKey"
          ref="robotMap" 
          :robot="activeRobot" 
          @selectedNodesChange="onSelectedNodesChange" 
        />
      </div>

      <div v-else-if="mode === 'manual'" class="flex justify-between items-start">
        <div class="flex flex-row items-center w-full">
          <Cctv 
            v-if="activeRobot"
            :robotSeq="activeRobot.seq" 
            :cameraType="'front'" 
            class="w-full h-[600px] bg-black rounded-lg shadow-lg flex items-center justify-center text-white text-xl" 
          />

          <div class="flex flex-col items-center ml-5 space-y-2">
            <button 
              @click="handleDirectionClick('up')"
              :class="[
                'w-14 h-14 rounded-full border-2 flex items-center justify-center text-2xl transition-all duration-200 shadow-md',
                activeArrows.has('ArrowUp') 
                  ? 'bg-blue-700 text-white border-blue-700 shadow-blue-300'
                  : 'border-blue-500 text-blue-500 hover:bg-blue-500 hover:text-white'
              ]"
            >
              ↑
            </button>
            <div class="flex space-x-2">
              <button 
                v-for="(direction, arrow) in {
                    left: 'ArrowLeft',
                    down: 'ArrowDown',
                    right: 'ArrowRight'
                  }"
                :key="arrow"
                @click="handleDirectionClick(direction)"
                :class="[
                  'w-14 h-14 rounded-full border-2 flex items-center justify-center text-2xl transition-all duration-200 shadow-md',
                  activeArrows.has(direction)
                    ? 'bg-blue-700 text-white border-blue-700 shadow-blue-300'
                    : 'border-blue-500 text-blue-500 hover:bg-blue-500 hover:text-white'
                ]"
              >
                {{ direction === 'ArrowLeft' ? '←' : direction === 'ArrowDown' ? '↓' : '→' }}
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch, nextTick } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/camera/Cctv.vue'
import RobotMap from '@/components/map/RobotMap.vue'
import ControlButtons from '@/components/map/ControlButtons.vue'
import axios from 'axios'

const robotsStore = useRobotsStore()
const selectedRobotSeq = ref('')
const selectedNodes = ref([])
const robotMap = ref(null)
const activeArrows = ref(new Set())
const isAutoMode = ref(true)
const mode = ref('auto')
const controlArea = ref(null)  // 컨트롤 영역에 대한 ref 추가

const activeRobot = computed(() => {
  return robotsStore.registered_robots.find(robot => 
    String(robot.seq) === String(selectedRobotSeq.value)
  ) || null
})

function onSelectedNodesChange(newNodes) {
  selectedNodes.value = [...newNodes]
}

function handleNavigate() {
  robotMap.value?.handleNavigate?.()
}

function handlePatrol() {
  robotMap.value?.handlePatrol?.()
}

function resetSelection() {
  robotMap.value?.resetSelection()  // RobotMap의 resetSelection 호출
  selectedNodes.value = []           // 부모의 선택 노드도 초기화
}

function handleTempStop() {
  robotMap.value?.handleTempStop?.()
}

async function handleKeyDown(event) {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key) && mode.value === 'manual') {
    event.preventDefault()
    activeArrows.value.add(event.key)

    // 방향키에 따른 엔드포인트 매핑
    const direction = {
      ArrowUp: 'up',
      ArrowDown: 'down',
      ArrowLeft: 'left',
      ArrowRight: 'right'
    }[event.key]

    try {
      // cmd_vel 토픽 발행
      await axios.post(`https://robocopbackendssafy.duckdns.org/api/v1/${selectedRobotSeq.value}/cmd_vel?direction=${direction}`)
    } catch (error) {
      console.error('로봇 제어 실패:', error)
    }
  }
}

async function handleKeyUp(event) {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
    activeArrows.value.delete(event.key)
    // 키를 떼었을 때는 별도의 정지 명령이 필요 없을 수 있음
    // 백엔드에서 자동으로 처리하는 것으로 보임
  }
}

// 버튼 클릭 핸들러 추가
async function handleDirectionClick(direction) {
  if (mode.value === 'manual') {
    try {
      console.log(`Sending ${direction} command to robot ${selectedRobotSeq.value}`)  // 요청 로그
      const response = await axios.post(`https://robocopbackendssafy.duckdns.org/api/v1/${selectedRobotSeq.value}/cmd_vel?direction=${direction}`)
      console.log('Response:', response.data)  // 응답 로그
    } catch (error) {
      console.error('로봇 제어 실패:', error)
    }
  }
}

function toggleMode() {
  mode.value = isAutoMode.value ? 'auto' : 'manual'
  
  // 수동 모드로 전환될 때 자동으로 포커스
  if (mode.value === 'manual') {
    // nextTick을 사용하여 DOM 업데이트 후 포커스
    nextTick(() => {
      controlArea.value?.focus()
    })
  }
}

const mapKey = ref(Date.now())

watch(selectedRobotSeq, (newVal, oldVal) => {
  if (newVal !== oldVal) {
    console.log('Robot changed:', newVal)
    // 로봇이 변경되면 선택된 노드들을 초기화
    resetSelection()
    
    // RobotMap 컴포넌트 재마운트를 위한 키 값 변경
    mapKey.value = Date.now()
  }
})

onMounted(() => {
  robotsStore.loadRobots()
  if (robotsStore.selectedRobot) {
    selectedRobotSeq.value = String(robotsStore.selectedRobot)
  }
  
  // 만약 초기 상태가 수동 모드라면 포커스
  if (mode.value === 'manual') {
    controlArea.value?.focus()
  }
})
</script>