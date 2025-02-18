<template>
  <div class="bg-white rounded-lg shadow-md p-5 font-sans">
    <div class="border-b pb-2 mb-4">
      <h4 class="text-lg font-bold">로봇 목록</h4>
    </div>
    
    <!-- 로봇 상태 목록 - 그리드 수정 -->
    <div :class="[
        'grid gap-6',
        'grid-cols-1',
        {'sm:grid-cols-2 md:grid-cols-3 xl:grid-cols-4': !robotsStore.leftSidebarCollapsed},
        {'md:grid-cols-2 lg:grid-cols-4 xl:grid-cols-5': robotsStore.leftSidebarCollapsed}
      ]"
    >
      <div 
        v-for="robot in visibleRobots" 
        :key="robot.seq"
        :class="[
          'bg-white rounded-lg p-4 shadow transition-all relative min-h-[250px]',
          {
            'opacity-40': webSocketConnected && !isActiveInWebSocket(robot),
            'hover:shadow-lg': !webSocketConnected || isActiveInWebSocket(robot)
          }
        ]"
      >
        <!-- 이전 컴포넌트의 나머지 내용은 동일하게 유지 -->
        <!-- X 버튼 -->
        <button 
          class="absolute top-2 right-2 text-gray-500 hover:text-gray-600 z-10" 
          @click="hideRobot(robot.seq)"
        >✖</button>

        <!-- 로봇 이름과 상태 표시를 나란히 배치 -->
        <div class="flex justify-between items-start mb-3">
          <div class="flex items-start gap-2">
            <div>
              <h3 class="text-lg font-semibold">{{ robot.nickname || robot.name }}</h3>
              <p class="text-sm text-gray-500">{{ robot.manufactureName }}</p>
            </div>
            
            <!-- 와이파이 아이콘 -->
            <div class="relative w-5 h-5">
              <svg viewBox="0 0 24 24" class="w-full h-full">
                <path 
                  :class="[
                    'transition-colors',
                    robot.networkHealth >= 80 ? 'fill-green-500' : 'fill-gray-200'
                  ]"
                  d="M12 3C7.95 3 4.21 4.34 1.2 6.6L3 9C5.5 7.12 8.62 6 12 6C15.38 6 18.5 7.12 21 9L22.8 6.6C19.79 4.34 16.05 3 12 3"
                />
                <path 
                  :class="[
                    'transition-colors',
                    robot.networkHealth >= 60 ? 'fill-green-500' : 'fill-gray-200'
                  ]"
                  d="M12 7C9.3 7 6.81 7.89 4.8 9.4L6.6 11.8C8.1 10.67 9.97 10 12 10C14.03 10 15.9 10.67 17.4 11.8L19.2 9.4C17.19 7.89 14.7 7 12 7"
                />
                <path 
                  :class="[
                    'transition-colors',
                    robot.networkHealth >= 40 ? 'fill-green-500' : 'fill-gray-200'
                  ]"
                  d="M12 11C10.4 11 8.85 11.45 7.4 12.2L9 14.5C9.9 13.83 10.93 13.5 12 13.5C13.07 13.5 14.1 13.83 15 14.5L16.6 12.2C15.15 11.45 13.6 11 12 11"
                />
                <path 
                  :class="[
                    'transition-colors',
                    robot.networkHealth >= 20 ? 'fill-green-500' : 'fill-gray-200'
                  ]"
                  d="M12 15C10.65 15 9.4 15.45 8.4 16.2L12 21L15.6 16.2C14.6 15.45 13.35 15 12 15"
                />
              </svg>
            </div>
          </div>

          <!-- 웹소켓 연결 상태 표시 -->
          <div v-if="webSocketConnected" 
            :class="[
              'px-2 py-1 rounded text-xs',
              isActiveInWebSocket(robot) ? 'bg-green-100 text-green-700' : 'bg-gray-100 text-gray-500'
            ]"
          >
            {{ isActiveInWebSocket(robot) ? '실시간 연결' : '연결 대기' }}
          </div>
        </div>

        <div class="space-y-2 mt-4">
          <div>
            <span class="text-sm text-gray-600">배터리</span>
            <div class="relative w-full h-4 bg-gray-200 rounded overflow-hidden">
              <div class="h-full" :class="getBatteryClass(robot.battery)" :style="{ width: robot.battery + '%' }"></div>
              <span class="absolute inset-0 flex justify-center items-center text-xs text-white font-semibold">{{ robot.battery }}%</span>
            </div>
          </div>
          <div>
            <span class="text-sm text-gray-600">현재 위치</span>
            <span class="block text-gray-800 font-medium">{{ robot.position ? `x: ${robot.position.x}, y: ${robot.position.y}` : '알 수 없음' }}</span>
          </div>
          <div>
            <span class="text-sm text-gray-600">작동 시간</span>
            <span class="block text-gray-800 font-medium">
              {{ getOperationTime(robot.startAt) }}
            </span>
          </div>
        </div>
        
        <div class="flex gap-2 mt-4">
          <button class="flex-1 py-2 rounded text-white text-sm bg-gray-700 hover:bg-gray-800" @click="returnRobot(robot.seq)">
            복귀 명령
          </button>
          <button 
            class="flex-1 py-2 rounded text-white text-sm" 
            :class="getEmergencyClass(robot.status)" 
            @click="handleStartStop(robot)"
          >
            {{ robot.status === 'navigating' ? '비상 정지' : '가동 시작' }}
          </button>
        </div>
        
        <button class="mt-2 w-full py-2 rounded bg-gray-900 text-white text-sm hover:bg-gray-800" @click="goToDetailPage(robot.seq)">
          상세 페이지
        </button>
      </div>
      
      <!-- 로봇 관리 버튼 -->
      <div class="flex flex-col items-center justify-center bg-gray-100 rounded-lg p-6 cursor-pointer min-h-[250px]" @click="robotsStore.openRobotManagementModal">
        <div class="text-6xl text-gray-400">+</div>
        <button class="mt-2 px-4 py-2 bg-gray-300 rounded hover:bg-gray-400 transition-colors">로봇 관리</button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue';
import { useRouter } from 'vue-router';
import { useRobotsStore } from '@/stores/robots';
import { useRobotCommandsStore } from '@/stores/robotCommands';
import { webSocketService } from '@/services/websocket';

const router = useRouter();
const robotsStore = useRobotsStore();
const robotCommandsStore = useRobotCommandsStore();
const hiddenRobots = ref([]);
// displayRobots에서 웹소켓 데이터가 병합된 로봇 정보를 가져옴
const robots = computed(() => robotsStore.displayRobots);
const visibleRobots = computed(() => 
  robots.value.filter(robot => !hiddenRobots.value.includes(robot.seq))
);
// ui적으로 로봇 숨기기
const hideRobot = (robotSeq) => hiddenRobots.value.push(robotSeq);

// 사이드바 상태에 따른 그리드 레이아웃 계산
const bothSidebarsCollapsed = computed(() => 
  robotsStore.leftSidebarCollapsed
);

const getBatteryClass = (battery) => {
  return {
    'bg-green-500': battery >= 30,
    'bg-red-500': battery < 30
  };
};

const getEmergencyClass = (status) => {
  return {
    'bg-red-600 hover:bg-red-700': status === 'navigating' || status === 'patrolling',
    'bg-emerald-500 hover:bg-emerald-600': status !== 'active'
  };
};

const getOperationTime = (startTime) => {
  if (!startTime) return '알 수 없음'
  
  const start = new Date(startTime)
  const now = new Date()
  const diff = now - start
  
  const hours = Math.floor(diff / (1000 * 60 * 60))
  const minutes = Math.floor((diff % (1000 * 60 * 60)) / (1000 * 60))
  
  return `${hours}시간 ${minutes}분`
};

const returnRobot = async (robotSeq) => {
  if (!robotSeq) return;
  try {
    await robotCommandsStore.homingCommand(robotSeq);
    alert('로봇이 복귀합니다.')
  } catch (err) {
    console.error('로봇 복귀 명령 에러:', err);
  }
};

const handleStartStop = async (robot) => {
  if (robot.status === 'navigating') {
    // 현재 가동 중이면 비상 정지 실행
    try {
      await robotCommandsStore.tempStopCommand(robot.seq);
      // 비상 정지 후 상태 업데이트 (예: 'active' 상태)
      robot.status = 'emergencyStopped';
      alert('로봇이 일시정지 되었습니다.')
    } catch (err) {
      console.error('비상 정지 명령 에러:', err);
    }
  } else {
    // 현재 가동 중이 아니면 navigateCommand 실행 (기본 목표는 로봇의 현재 위치 또는 [0,0])
    try {
      await robotCommandsStore.resumeCommand(robot.seq);
      // 가동 시작 후 상태 업데이트
      robot.status = 'navigating';
      alert('활동을 시작합니다.')
    } catch (err) {
      console.error('가동 시작 명령 에러:', err);
    }
  }
};

const goToDetailPage = (robotSeq) => router.push(`/${robotSeq}`);

// 웹소켓으로 활성화된 로봇인지 확인 (manufactureName 기준으로 수정)
const isActiveInWebSocket = (robot) => {
  return /^robot_\d+$/.test(robot.manufactureName) && robot.isActive;
}

// 웹소켓 메시지 핸들러
const handleWebSocketMessage = (message) => {
  if (message.type === 'ros_topic') {
    const robotId = message.topic.split('/')[1]
    
    switch (message.topic) {
      case `/${robotId}/status`:
        robotsStore.updateRobotWebSocketData(robotId, message.data, 'status')
        break

      case `/${robotId}/utm_pose`:
        if (message.messageData?.position) {
          // 데이터 타입 확인 및 가공
          console.log('UTM Position Types:', {
            x: typeof message.messageData.position.x,
            y: typeof message.messageData.position.y,
            rawX: message.messageData.position.x,
            rawY: message.messageData.position.y
          })

          // position 객체 구조를 DB 형식에 맞게 변환
          const positionData = {
            x: Number(message.messageData.position.x.toFixed(4)),  // 소수점 4자리까지만
            y: Number(message.messageData.position.y.toFixed(4))   // Number로 변환
          }

          const formattedData = {
            position: positionData,
            rawPosition: message.messageData.position,
            orientation: message.messageData.orientation
          }
          robotsStore.updateRobotWebSocketData(robotId, formattedData, 'utm_pose')
        }
        break
    }
  }
}

onMounted(() => {
  // 핸들러만 등록
  webSocketService.registerHandler('ros_topic', handleWebSocketMessage)
})

onUnmounted(() => {
  // 핸들러만 제거
  webSocketService.removeHandler('ros_topic', handleWebSocketMessage)
})
</script>