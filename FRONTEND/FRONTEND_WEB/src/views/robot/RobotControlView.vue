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
          @resume="handleResume"
        />

        <RobotMap 
          v-if="activeRobot"
          :key="mapKey"
          ref="robotMap" 
          :robot="activeRobot"
          :isManualMode="mode === 'manual'"
          @selectedNodesChange="onSelectedNodesChange" 
        />
      </div>

      <div v-else-if="mode === 'manual'" class="flex justify-between items-start">
        <div class="flex flex-row items-center w-full">
          <Cctv 
            v-if="activeRobot"
            :robotSeq="String(activeRobot.seq)" 
            :cameraType="'front'" 
            class="w-full h-[530px] bg-black rounded-lg shadow-lg flex items-center justify-center text-white text-xl" 
          />

          <div class="flex flex-col items-center ml-5 space-y-2">
            <button 
              @click="handleDirectionClick('ArrowUp')"
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
                v-for="direction in ['ArrowLeft', 'ArrowDown', 'ArrowRight']"
                :key="direction"
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
            <button 
              @click="handleDirectionClick(' ')"
              :class="[
                'w-44 h-10 rounded-lg border-2 flex items-center justify-center text-sm transition-all duration-200 shadow-md mt-2',
                activeArrows.has(' ') 
                  ? 'bg-blue-700 text-white border-blue-700 shadow-blue-300'
                  : 'border-blue-500 text-blue-500 hover:bg-blue-500 hover:text-white'
              ]"
            >
              Spacebar
            </button>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch, nextTick, onBeforeUnmount } from 'vue'
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
const controlArea = ref(null)

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
  robotMap.value?.resetSelection()
  selectedNodes.value = []
}

function handleTempStop() {
  robotMap.value?.handleTempStop?.()
}

function handleResume() {
  robotMap.value?.handleResume?.()
}

async function handleKeyDown(event) {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(event.key) && mode.value === 'manual') {
    event.preventDefault()
    activeArrows.value.add(event.key)

    // 방향키에 따른 엔드포인트 매핑
    const direction = {
      ArrowUp: 'UP',
      ArrowDown: 'DOWN',
      ArrowLeft: 'LEFT',
      ArrowRight: 'RIGHT',
      ' ': 'SPACE'
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
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(event.key)) {
    activeArrows.value.delete(event.key)
  }
}

// 버튼 클릭 핸들러 추가
async function handleDirectionClick(key) {
  if (mode.value === 'manual') {
    try {
      // 키 입력에 따른 direction 매핑
      const directionMap = {
        'ArrowUp': 'UP',
        'ArrowDown': 'DOWN',
        'ArrowLeft': 'LEFT',
        'ArrowRight': 'RIGHT',
        ' ': 'SPACE'
      }
      
      const direction = directionMap[key]
      activeArrows.value.add(key)  // UI 상태 업데이트

      console.log(`Sending ${direction} command to robot ${selectedRobotSeq.value}`)
      const response = await axios.post(
        `https://robocopbackendssafy.duckdns.org/api/v1/${selectedRobotSeq.value}/cmd_vel?direction=${direction}`
      )
      console.log('Response:', response.data)

      // 버튼 클릭의 경우 짧은 시간 후 상태 해제
      setTimeout(() => {
        activeArrows.value.delete(key)
      }, 200)
    } catch (error) {
      console.error('로봇 제어 실패:', error)
      activeArrows.value.delete(key)  // 에러 시 상태 해제
    }
  }
}

async function toggleMode() {
  try {
    const newMode = isAutoMode.value ? 'resume' : 'manual'
    const response = await axios.post(
      `https://robocopbackendssafy.duckdns.org/api/v1/${selectedRobotSeq.value}/call-service/${newMode}`
    )
    
    if (response.status === 200) {
      mode.value = isAutoMode.value ? 'auto' : 'manual'
      
      // 수동 모드로 전환될 때 자동으로 포커스
      if (mode.value === 'manual') {
        nextTick(() => {
          controlArea.value?.focus()
        })
      }
    } else {
      // API 요청이 실패하면 토글 상태를 원래대로 되돌림
      isAutoMode.value = !isAutoMode.value
      console.error('모드 변경 실패')
    }
  } catch (error) {
    // 에러 발생 시 토글 상태를 원래대로 되돌림
    isAutoMode.value = !isAutoMode.value
    console.error('모드 변경 중 에러 발생:', error)
  }
}

const mapKey = ref(Date.now())

watch(selectedRobotSeq, async (newVal, oldVal) => {
  if (newVal !== oldVal) {
    console.log('Robot changed:', newVal)
    resetSelection()
    mapKey.value = Date.now()
    
    // 로봇이 선택되면 자동으로 resume 모드로 설정
    if (newVal) {
      try {
        await axios.post(
          `https://robocopbackendssafy.duckdns.org/api/v1/${newVal}/call-service/resume`
        )
        isAutoMode.value = true
        mode.value = 'auto'
      } catch (error) {
        console.error('초기 모드 설정 실패:', error)
      }
    }
  }
})

onMounted(() => {
  robotsStore.loadRobots()
  if (robotsStore.selectedRobot) {
    selectedRobotSeq.value = String(robotsStore.selectedRobot)
  }
  
  // main-content 요소의 overflow 설정 변경
  const mainContent = document.querySelector('.main-content')
  if (mainContent) {
    mainContent.style.overflow = 'hidden'
  }
})

onBeforeUnmount(() => {
  // 컴포넌트 언마운트 시 원래 상태로 복구
  const mainContent = document.querySelector('.main-content')
  if (mainContent) {
    mainContent.style.overflow = 'auto'
  }
})
</script>