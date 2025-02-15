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
        <div class="font-medium mb-1">최근 메시지:</div>
        <div v-if="lastTestMessage?.data" class="bg-gray-800 text-white p-2 rounded font-mono text-sm">
          {{ lastTestMessage.data }}
          <div class="text-xs text-gray-400 text-right mt-1">{{ lastMessageTime }}</div>
        </div>
        <div v-else class="text-gray-500 italic">수신된 메시지 없음</div>
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
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue';
import { webSocketService } from '@/services/websocket';
import { useRobotsStore } from '@/stores/robots';
import RobotList from '@/components/dashboard/RobotList.vue';
import RobotManagement from '@/components/dashboard/RobotManagement.vue';
import RobotRegistration from '@/components/dashboard/RobotRegistration.vue';
import StatisticsView from '@/views/statistics/StatisticsView.vue';

// 모니터링 컴포넌트 순서 관리
const robotsStore = useRobotsStore()
const robots = computed(() => robotsStore.registered_robots)
const webSocketConnected = ref(false)
const lastTestMessage = ref(null)
const lastMessageTime = ref('')
const messageInput = ref('')
const connectionMode = computed(() => 
  webSocketConnected.value ? '실시간 모드' : 'API 모드'
)

// 탭 관리
const activeTab = ref('robotList') // 기본값: 로봇 목록 탭
const tabs = ref([
  { name: 'robotList', label: '로봇 목록' },
  { name: 'statistics', label: '통계' }
])

// WebSocket 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('wss://robocopbackendssafy.duckdns.org/ws/test')
    webSocketConnected.value = true
    console.log('웹소켓 연결 성공');

    // 테스트 메시지 구독
    webSocketService.subscribe('test_message', (data) => {
      console.log('수신된 원본 데이터:', data);
      
      try {
        const messageData = typeof data === 'string' ? JSON.parse(data) : data;
        
        if (messageData.type === 'test_message') {
          lastTestMessage.value = messageData;
          lastMessageTime.value = new Date().toLocaleTimeString();
          console.log('처리된 테스트 메시지:', messageData);
        } else {
          console.warn('알 수 없는 메시지 형식:', messageData);
        }
      } catch (e) {
        console.error('메시지 처리 중 오류:', e);
        console.log('처리 실패한 원본 데이터:', data);
      }
    });

    // 로봇 상태 구독
    webSocketService.subscribe('monitoring/robots', (data) => {
      console.log('로봇 상태 데이터:', data);
      robotsStore.updateRobotsData(data)
    })

  } catch (error) {
    console.error('WebSocket 연결 실패:', error)
    webSocketConnected.value = false
    // API 모드로 전환
    robotsStore.startPolling()
  }
};

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
  };
}

onMounted(() => {
  setupWebSocket();
});

onUnmounted(() => {
  if (webSocketService.isConnected()) {
    webSocketService.disconnect()
  }
  robotsStore.stopPolling()
})

</script>

<style scoped>
.connection-badge {
  @apply px-2 py-1 rounded text-white text-sm;
}
</style>