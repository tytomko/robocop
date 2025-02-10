<template>
  <div class="bg-white rounded-lg shadow-md p-5 font-sans">
    <div class="border-b pb-2 mb-4">
      <h4 class="text-lg font-bold">로봇 목록</h4>
    </div>
    
    <!-- 로봇 상태 목록 -->
    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      <div v-for="robot in visibleRobots" :key="robot.seq" class="relative bg-white rounded-lg p-4 border shadow-sm">
        <!-- X 버튼 -->
        <button class="absolute top-2 right-2 text-gray-500 hover:text-gray-600" @click="hideRobot(robot.seq)">✖</button>

        <!-- 로봇 이름과 상태 표시를 나란히 배치 -->
        <div class="flex items-center mt-2">
          <span class="font-bold mr-2">{{ robot.nickname || robot.name }}</span>
          <span class="px-2 py-1 text-xs font-semibold rounded-full" :class="getStatusClass(robot.status)">
            {{ getStatusLabel(robot.status) }}
          </span>
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
            <span class="block text-gray-800 font-medium">{{ robot.position }}</span>
          </div>
          <div>
            <span class="text-sm text-gray-600">IP 주소</span>
            <span class="block text-gray-800 font-medium">{{ robot.ipAddress }}</span>
          </div>
        </div>
        
        <div class="flex gap-2 mt-4">
          <button class="flex-1 py-2 rounded text-white text-sm bg-gray-700 hover:bg-gray-800" @click="returnRobot(robot.seq)">
            복귀 명령
          </button>
          <button class="flex-1 py-2 rounded text-white text-sm" :class="getEmergencyClass(robot.status)" @click="emergencyStop(robot.seq)">
            {{ robot.status === 'navigating' ? '비상 정지' : '가동 시작' }}
          </button>
        </div>
        
        <button class="mt-2 w-full py-2 rounded bg-gray-900 text-white text-sm hover:bg-gray-800" @click="goToDetailPage(robot.seq)">
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

const visibleRobots = computed(() => robots.value.filter(robot => !hiddenRobots.value.includes(robot.seq)));

const hideRobot = (robotSeq) => hiddenRobots.value.push(robotSeq);

const getStatusClass = (status) => {
  return {
    'bg-green-500 text-white': status === 'charging',
    'bg-blue-500 text-white': status === 'patrolling' || status === 'navigating',
    'bg-red-500 text-white': status === 'emergencyStopped' || status === 'error',
    'bg-gray-500 text-white': status === 'waiting',
    'bg-black text-white': status === 'homing'
  };
};

const getStatusLabel = (status) => ({
  navigating: '이동 중', charging: '충전 중', emergencyStopped: '정지 중', error: '고장',
  waiting: '대기 중', homing: '복귀 중', patrolling: '순찰 중'
}[status] || status);

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

const returnRobot = async (robotSeq) => {
  if (!robotSeq) return;
  try {
    const robot = robotsStore.registered_robots.find((r) => r.seq === robotSeq);
    if (robot) robot.status = 'homing';
  } catch (err) {
    console.error('로봇 복귀 명령 에러:', err);
  }
};

const emergencyStop = async (robotSeq) => {
  if (!robotSeq) return;
  try {
    const robot = robotsStore.registered_robots.find((r) => r.seq === robotSeq);
    if (robot) robot.status = robot.status === 'navigating' ? 'emergencyStopped' : 'navigating';
  } catch (err) {
    console.error('비상 정지 명령 에러:', err);
  }
};

const goToDetailPage = (robotSeq) => router.push(`/${robotSeq}`);
</script>
