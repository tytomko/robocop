<template>
  <div class="h-full flex flex-col gap-1.5 p-5 overflow-y-auto bg-gray-100">
    <!-- 웹소켓 테스트 섹션 -->
    <div class="bg-white rounded-lg p-4 mb-4 shadow">
      <div class="flex justify-between items-center mb-2">
        <h3 class="text-lg font-semibold">WebSocket 테스트</h3>
        <div class="flex items-center gap-2">
          <span class="text-sm">{{ connectionMode }}</span>
          <span :class="['connection-badge', webSocketConnected ? 'bg-green-500' : 'bg-red-500']">
            {{ webSocketConnected ? 'WebSocket' : 'REST API' }}
          </span>
        </div>
      </div>
      
      <!-- 웹소켓 메시지 표시 영역 -->
      <div v-if="webSocketConnected" class="bg-gray-100 rounded p-3 mb-3">
        <div class="font-medium mb-1">Latest Message:</div>
        <div v-if="lastTestMessage" class="bg-gray-800 text-white p-2 rounded font-mono text-sm">
          <div v-if="lastTestMessage.type === 'ros_topic'" class="mb-2">
            <div class="text-blue-400">Topic: {{ lastTestMessage.topic }}</div>
            <div v-if="lastTestMessage.data && lastTestMessage.data.msg">
              <pre class="whitespace-pre-wrap">{{ JSON.stringify(lastTestMessage.data.msg, null, 2) }}</pre>
            </div>
          </div>
          <div v-else>
            {{ lastTestMessage }}
          </div>
          <div class="text-xs text-gray-400 text-right mt-1">{{ lastMessageTime }}</div>
        </div>
        <div v-else class="text-gray-500 italic">No messages received</div>
      </div>

      <!-- 메시지 입력 영역 -->
      <div v-if="webSocketConnected" class="flex gap-2">
        <input 
          v-model="messageInput"
          @keyup.enter="sendMessage"
          placeholder="메시지를 입력하세요..."
          class="flex-1 px-3 py-2 border rounded"
        />
        <button 
          @click="sendMessage"
          class="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
        >
          전송
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
      <RobotList v-if="activeTab === 'robotList'" />
      <StatisticsView v-if="activeTab === 'statistics'" />
    </div>

    <!-- 로봇 관리 모달 -->
    <RobotManagement
      :show="robotsStore.showRobotManagementModal"
      :robots="robots"
      @close="robotsStore.closeRobotManagementModal"
      @openAddRobotModal="robotsStore.openAddRobotModal"
      @setBreakdown="robotsStore.setBreakdown"
      @setActive="robotsStore.setActive"
    />

    <!-- 로봇 등록 모달 -->
    <RobotRegistration
      :show="robotsStore.showModal"
      :newRobot="robotsStore.newRobot"
      @handleAddRobot="robotsStore.handleAddRobot"
      @close="robotsStore.closeModal"
    />

    <!-- 로봇 상태 표시 -->
    <div class="status-panel">
      <h3>로봇 상태</h3>
      <p>상태: {{ robotStatus?.status || '연결 대기중' }}</p>
      <p>속도: {{ robotSpeed.toFixed(2) }} m/s</p>
      <p>방향: {{ robotHeading.toFixed(2) }}°</p>
    </div>

    <!-- 위치 정보 표시 -->
    <div class="position-panel" v-if="robotPosition">
      <h3>로봇 위치</h3>
      <p>X: {{ robotPosition.pose.position.x.toFixed(2) }}</p>
      <p>Y: {{ robotPosition.pose.position.y.toFixed(2) }}</p>
      <p>Z: {{ robotPosition.pose.position.z.toFixed(2) }}</p>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { webSocketService } from '@/services/websocket'
import { useRobotsStore } from '@/stores/robots'
import RobotList from '@/components/dashboard/RobotList.vue'
import RobotManagement from '@/components/dashboard/RobotManagement.vue'
import RobotRegistration from '@/components/dashboard/RobotRegistration.vue'
import StatisticsView from '@/views/statistics/StatisticsView.vue'

const robotsStore = useRobotsStore()
const robots = computed(() => robotsStore.registered_robots)
const webSocketConnected = computed(() => webSocketService.isConnected.value)
const lastTestMessage = ref(null)
const lastMessageTime = ref('')
const messageInput = ref('')
const connectionMode = computed(() => 
  webSocketConnected.value ? '실시간 모드' : 'API 모드'
)

// 탭 관리
const activeTab = ref('robotList')
const tabs = ref([
  { name: 'robotList', label: '로봇 목록' },
  { name: 'statistics', label: '통계' }
])

const robotStatus = ref(null)
const robotPosition = ref(null)
const robotSpeed = ref(0)
const robotHeading = ref(0)

// 웹소켓 메시지 핸들러
const handleWebSocketMessage = (message) => {
  if (message.type === 'ros_topic') {
    switch (message.topic) {
      case '/robot_1/status':
        robotStatus.value = message.data.msg
        break
      case '/robot_1/utm_pose':
        robotPosition.value = message.data.msg
        break
      case '/robot_1/speed_mps':
        robotSpeed.value = message.data.msg.data
        break
      case '/robot_1/heading':
        robotHeading.value = message.data.msg.data
        break
    }
  }
}

// 웹소켓 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('wss://robocopbackendssafy.duckdns.org/ws/test')
    
    // 메시지 핸들러 등록
    webSocketService.registerHandler('ros_topic', handleWebSocketMessage)
    
    // 프론트엔드 토픽 구독
    const frontendTopics = [
      '/robot_1/status',
      '/robot_1/utm_pose',
      '/robot_1/speed_mps',
      '/robot_1/heading'
    ]

    frontendTopics.forEach(topic => {
      webSocketService.subscribe(topic, (data) => {
        lastTestMessage.value = {
          type: 'ros_topic',
          topic: topic,
          data: data
        }
        lastMessageTime.value = new Date().toLocaleTimeString()
      })
    })

  } catch (error) {
    console.error('WebSocket connection failed:', error)
    robotsStore.startPolling()
  }
}

// 메시지 전송
const sendMessage = async () => {
  if (!messageInput.value.trim()) return
  
  try {
    await webSocketService.send('user_message', {
      text: messageInput.value
    })
    messageInput.value = ''
  } catch (error) {
    console.error('메시지 전송 실패:', error)
  }
}

onMounted(() => {
  setupWebSocket()
})

onUnmounted(() => {
  webSocketService.disconnect()
  robotsStore.stopPolling()
})
</script>

<style scoped>
.connection-badge {
  @apply px-2 py-1 rounded text-white text-sm;
}

.status-panel {
  @apply bg-white rounded-lg p-4 mb-4 shadow;
}

.position-panel {
  @apply bg-white rounded-lg p-4 mb-4 shadow;
}
</style>
