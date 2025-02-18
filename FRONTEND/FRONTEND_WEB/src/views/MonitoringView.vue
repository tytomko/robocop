<template>
  <div class="h-full flex flex-col gap-1.5 p-5 overflow-y-auto bg-gray-100">
    <!-- 연결 상태 표시 -->
    <div class="bg-white rounded-lg p-3 mb-4 shadow">
      <div class="flex justify-between items-center">
        <div class="flex items-center gap-2">
          <span class="font-semibold">연결 상태:</span>
          <span :class="[
            'px-2 py-1 rounded text-white text-sm',
            webSocketConnected ? 'bg-green-500' : 'bg-red-500'
          ]">
            {{ connectionStatus }}
          </span>
        </div>
        <div class="text-sm text-gray-600">
          {{ webSocketConnected ? '실시간 데이터 수신 중' : 'DB에서 데이터 폴링 중' }}
        </div>
      </div>
    </div>

    <div class="container mx-auto px-4">
    <!-- 기존 MonitoringView 내용 -->

    <!-- 테스트용 버튼 -->
    <div class="fixed bottom-4 right-4">
      <button 
        @click="triggerAlert"
        class="bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded-md shadow-lg"
      >
        테스트 경보 발생
      </button>
    </div>
  </div>

    <div class="border-b pb-2 mb-4">
      <h2 class="text-2xl font-bold text-gray-800">실시간 모니터링</h2>
    </div>

    <!-- 탭 메뉴 -->
    <div class="flex border-b">
      <button
        v-for="tab in tabs"
        :key="tab.name"
        @click="activeTab = tab.name"
        class="px-6 py-3 text-gray-600 font-semibold transition-all"
        :class="{
          'border-b-4 border-blue-500 text-blue-600': activeTab === tab.name,
          'hover:text-blue-500': activeTab !== tab.name
        }"
      >
        {{ tab.label }}
      </button>
    </div>

    <!-- 탭 콘텐츠 -->
    <div class="mt-4">
      <RobotMap v-if="activeTab === 'robotMap'" />
      <RobotList v-if="activeTab === 'robotList'" />
      <StatisticsView v-if="activeTab === 'statistics'" />
    </div>

    <!-- 로봇 관리 모달 -->
    <RobotManagement
      :show="robotsStore.showRobotManagementModal"
      :robots="robots"
      @close="robotsStore.closeRobotManagementModal"
      @openAddRobotModal="robotsStore.openAddRobotModal"
    />

    <!-- 로봇 등록 모달 -->
    <RobotRegistration
      :show="robotsStore.showModal"
      :newRobot="robotsStore.newRobot"
      @handleAddRobot="robotsStore.handleAddRobot"
      @close="robotsStore.closeModal"
    />
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue';
import { webSocketService } from '@/services/websocket';
import { useRobotsStore } from '@/stores/robots';
import RobotList from '@/components/dashboard/RobotList.vue';
import RobotManagement from '@/components/dashboard/RobotManagement.vue';
import RobotRegistration from '@/components/dashboard/RobotRegistration.vue';
import RobotMap from '@/components/map/RobotMap.vue';
import StatisticsView from '@/views/statistics/StatisticsView.vue';

// 모니터링 컴포넌트 순서 관리
const robotsStore = useRobotsStore()
const robots = computed(() => robotsStore.registered_robots)
const webSocketConnected = computed(() => robotsStore.webSocketConnected)
const connectionStatus = computed(() => 
  webSocketConnected.value ? '실시간 모드' : 'API 모드'
)

// 탭 관리
const activeTab = ref('robotMap') // 기본값: 로봇 목록 탭
const tabs = ref([
  { name: 'robotMap', label: '실시간 로봇 위치' },
  { name: 'robotList', label: '로봇 목록' },
  { name: 'statistics', label: '통계' }
])

// 웹소켓 메시지 핸들러
const handleWebSocketMessage = (message) => {
  if (message.type === 'ros_topic') {
    const robotId = message.topic.split('/')[1]
    
    switch (message.topic) {
      case `/${robotId}/status`:
        robotsStore.updateRobotWebSocketData(robotId, message.data)
        break
      case `/${robotId}/utm_pose`:
        const poseData = message.messageData || message.data
        if (poseData?.position) {
          robotsStore.updateRobotWebSocketData(robotId, {
            position: `x: ${poseData.position.x.toFixed(2)}, y: ${poseData.position.y.toFixed(2)}`
          })
        }
        break
    }
  }
}

// 단순화된 테스트 함수
const triggerAlert = () => {
  robotsStore.alerts = true;
};

onMounted(() => {
  // 핸들러만 등록
  webSocketService.registerHandler('ros_topic', handleWebSocketMessage)
})

onUnmounted(() => {
  // 핸들러만 제거
  webSocketService.removeHandler('ros_topic', handleWebSocketMessage)
})
</script>