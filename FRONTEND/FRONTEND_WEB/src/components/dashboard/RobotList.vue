<template>
  <div class="bg-white rounded-lg shadow-md p-4">
    <div class="border-b pb-2 mb-4">
      <h4 class="text-lg font-semibold">로봇 목록</h4>
    </div>
    
    <!-- 로봇 상태 목록 -->
    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      <div v-for="robot in visibleRobots" :key="robot.id" class="relative bg-white rounded-lg p-4 border shadow-sm">
        <!-- X 버튼 -->
        <button class="absolute top-2 right-2 text-gray-500 hover:text-gray-800" @click="hideRobot(robot.id)">✖</button>

        <!-- 상태 표시 위치 조정 -->
        <span class="absolute top-2 right-10 px-2 py-1 text-xs font-semibold rounded-full" :class="getStatusClass(robot.status)">
          {{ getStatusLabel(robot.status) }}
        </span>
        
        <div class="flex justify-between items-center mb-4">
          <span class="font-bold">{{ robot.nickname || robot.name }}</span>
        </div>
        
        <div class="space-y-2">
          <div>
            <span class="text-sm text-gray-600">배터리</span>
            <div class="relative w-full h-4 bg-gray-200 rounded overflow-hidden">
              <div class="h-full" :class="getBatteryClass(robot.battery)" :style="{ width: robot.battery + '%' }"></div>
              <span class="absolute inset-0 flex justify-center items-center text-xs text-white font-semibold">{{ robot.battery }}%</span>
            </div>
          </div>
          <div>
            <span class="text-sm text-gray-600">현재 위치</span>
            <span class="block text-gray-800 font-medium">{{ robot.location }}</span>
          </div>
          <div>
            <span class="text-sm text-gray-600">IP 주소</span>
            <span class="block text-gray-800 font-medium">{{ robot.ipAddress }}</span>
          </div>
        </div>
        
        <div class="flex gap-2 mt-4">
          <button class="flex-1 py-2 rounded text-white text-sm" :class="{'bg-gray-500 hover:bg-gray-600': robot.status !== 'returning', 'bg-black': robot.status === 'returning'}" @click="returnRobot(robot.id)">
            복귀 명령
          </button>
          <button class="flex-1 py-2 rounded text-white text-sm" :class="getEmergencyClass(robot.status)" @click="emergencyStop(robot.id)">
            {{ robot.status === 'active' ? '비상 정지' : '가동 시작' }}
          </button>
        </div>
        
        <button class="mt-2 w-full py-2 rounded bg-black text-white text-sm hover:bg-gray-800" @click="goToDetailPage(robot.id)">
          상세 페이지
        </button>
      </div>
      
      <!-- 로봇 관리 버튼 -->
      <div class="flex flex-col items-center justify-center bg-gray-100 rounded-lg p-6 cursor-pointer" @click="robotsStore.openRobotManagementModal">
        <div class="text-6xl text-gray-400">+</div>
        <button class="mt-2 px-4 py-2 bg-gray-300 rounded">로봇 관리</button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed } from 'vue';
import { useRouter } from 'vue-router';
import { useRobotsStore } from '@/stores/robots';

const router = useRouter();
const robotsStore = useRobotsStore();
const robots = computed(() => robotsStore.registered_robots);
const hiddenRobots = ref([]);

const visibleRobots = computed(() => robots.value.filter(robot => !hiddenRobots.value.includes(robot.id)));

const hideRobot = (robotId) => hiddenRobots.value.push(robotId);

const getStatusClass = (status) => {
  return {
    'bg-green-500 text-white': status === 'active',
    'bg-blue-500 text-white': status === 'charging',
    'bg-red-500 text-white': status === 'stopped' || status === 'error',
    'bg-gray-500 text-white': status === 'idle',
    'bg-black text-white': status === 'returning' || status === 'breakdown'
  };
};

const getStatusLabel = (status) => ({
  active: '활동 중', charging: '충전 중', stopped: '정지 중', error: '오류 발생',
  idle: '대기 중', returning: '복귀 중', breakdown: '고장'
}[status] || status);

const getBatteryClass = (battery) => {
  return {
    'bg-green-500': battery >= 80,
    'bg-yellow-500': battery >= 50,
    'bg-orange-500': battery >= 30,
    'bg-red-500': battery < 30
  };
};

const getEmergencyClass = (status) => {
  return {
    'bg-red-600 hover:bg-red-700': status === 'active',
    'bg-green-600 hover:bg-green-700': status !== 'active'
  };
};

const returnRobot = async (robotId) => {
  if (!robotId) return;
  try {
    const robot = robotsStore.registered_robots.find((r) => r.id === robotId);
    if (robot) robot.status = 'returning';
  } catch (err) {
    console.error('로봇 복귀 명령 에러:', err);
  }
};

const emergencyStop = async (robotId) => {
  if (!robotId) return;
  try {
    const robot = robotsStore.registered_robots.find((r) => r.id === robotId);
    if (robot) robot.status = robot.status === 'active' ? 'stopped' : 'active';
  } catch (err) {
    console.error('비상 정지 명령 에러:', err);
  }
};

const goToDetailPage = (robotId) => router.push(`/${robotId}`);
</script>
