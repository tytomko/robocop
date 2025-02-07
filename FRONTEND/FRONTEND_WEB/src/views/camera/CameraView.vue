<template>
  <div class="p-5 h-[90vh] overflow-y-auto">
    <!-- 실시간 모니터링 섹션 -->
    <div class="flex flex-col gap-5 h-auto min-h-full">
      <div class="p-2 border-b border-gray-200 mt-2">
        <!-- 로봇 선택 -->
        <select
          v-model="selectedRobots"
          class="w-full p-2 border border-gray-300 rounded bg-white text-sm appearance-none bg-no-repeat bg-right pr-8"
          @change="handleRobotSelection"
          multiple
        >
          <option value="">로봇 선택</option>
          <option v-for="robot in robotsStore.registered_robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
      </div>

      <!-- 선택된 로봇 카메라들 -->
      <div v-if="selectedRobots.length > 0" class="flex flex-wrap gap-4">
        <Cctv
          v-for="robotId in selectedRobots"
          :key="robotId"
          :cameraName="'카메라 ' + robotId"
          :cameraStatus="'연결 대기 중'"
          :streamInfo="streamInfoMap[robotId]"
        />
      </div>
      <div v-else class="p-5 text-center text-gray-500">로봇을 선택해주세요</div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import { useRobotsStore } from '@/stores/robots'
import Cctv from '@/components/camera/Cctv.vue'

// robotsStore 활용하기
const robotsStore = useRobotsStore()

// 선택된 로봇들
const selectedRobots = ref([])
// 각 로봇의 스트림 정보
const streamInfoMap = ref({})

// 로봇 선택 후 상태 변경
const handleRobotSelection = () => {
  // 선택된 로봇들에 대해 스트림 정보 등을 갱신
  selectedRobots.value.forEach(robotId => {
    streamInfoMap.value[robotId] = robotsStore.getStreamInfo(robotId)
  })
}

// 로봇 데이터 불러오기
onMounted(() => {
  robotsStore.loadRobots() // 로봇 데이터 불러오기
  const savedRobots = localStorage.getItem('selectedRobots')
  if (savedRobots) {
    selectedRobots.value = JSON.parse(savedRobots)
  }
})

// selectedRobots가 변경될 때마다 로컬 스토리지에 저장
watch(selectedRobots, (newRobots) => {
  localStorage.setItem('selectedRobots', JSON.stringify(newRobots))
})
</script>
