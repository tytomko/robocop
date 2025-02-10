<template>
  <div class="h-full flex flex-col gap-1.5 p-5 overflow-y-auto bg-gray-100">
    <div class="border-b pb-2 mb-4">
      <h2 class="text-2xl font-bold text-gray-800">실시간 모니터링</h2>
    </div>

    <!-- 등록된 로봇 리스트-->
    <RobotList />

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
import { computed, onMounted, onUnmounted } from 'vue'
import { webSocketService } from '@/services/websocket'
import { useRobotsStore } from '@/stores/robots'
import RobotList from '@/components/dashboard/RobotList.vue';
import RobotManagement from '@/components/dashboard/RobotManagement.vue';
import RobotRegistration from '@/components/dashboard/RobotRegistration.vue';

// 모니터링 컴포넌트 순서 관리
const robotsStore = useRobotsStore()
const robots = computed(() => robotsStore.registered_robots)

// WebSocket 연결 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/ws');

    // 로봇 상태 구독
    webSocketService.subscribe('monitoring/robots', (data) => {
      console.log('로봇 상태 업데이트:', data);
      loadRobotsWithNicknames(); // 상태 업데이트 후 닉네임 유지
    });

  } catch (error) {
    console.error('WebSocket 연결 실패:', error);
  }
};

// 로봇 데이터 불러오기 및 닉네임 설정
const loadRobotsWithNicknames = async () => {
  robots.value = robotsStore.registered_robots.map(robot => {
    return {
      ...robot,
      nickname: localStorage.getItem(`robot_nickname_${robot.seq}`) || robot.name
    };
  });
};

onMounted(() => {
  loadRobotsWithNicknames(); // 닉네임 불러오기
  setupWebSocket();
});

onUnmounted(() => {
  if (webSocketService.isConnected()) {
    webSocketService.disconnect()
  }
})
</script>
