<template>
  <div class="bg-gray-100 p-4 rounded-lg shadow">
    <!-- 기본 정보 -->
    <div class="mb-4">
      <h3 class="text-lg font-semibold mb-2">기본 정보</h3>
      <p><strong>상태:</strong> {{ getStatusLabel(robot.status) }}</p>
      <p><strong>배터리:</strong> {{ robot.battery.level }}%</p>
      <p><strong>네트워크 상태:</strong> {{ robot.networkHealth }}% ({{ robot.networkStatus }})</p>
      <!-- <p><strong>CPU 온도:</strong> {{ robot.cpuTemp }}°C</p>
      <p><strong>속도: {{ robot.motion }}</strong></p> -->
      <p><strong>작동 시작 시간:</strong> {{ getOperationTime(robot.startAt) }}</p>
    </div>
    <hr class="border-gray-300 my-4">


    <!-- 센서 데이터 -->
    <!-- <div class="mb-4">
      <h3 class="text-lg font-semibold mb-2">센서 데이터</h3>
      <div class="flex flex-wrap gap-2">
        <span v-for="(status, sensor) in robot.sensors" 
              :key="sensor"
              class="px-2 py-1 rounded text-white text-sm"
              :class="getSensorClass(status)">
          {{ getSensorLabel(sensor) }}: {{ status }}
        </span>
      </div>
    </div>
    <hr class="border-gray-300 my-4"> -->

    <!-- 3D 라이다 정보 -->
    <div>
      <div class="flex justify-between items-center mb-2">
        <h3 class="text-lg font-semibold">3D 라이다</h3>
        <div class="text-sm">
          <span class="text-gray-600">포인트: </span>
          <span class="font-medium">{{ lidarPoints }}</span>
          <span class="mx-2">|</span>
          <span class="text-gray-600">갱신: </span>
          <span class="font-medium">{{ formatTimestamp(lastUpdate) }}</span>
        </div>
      </div>
      <div class="mb-4">
        <LidarViewer 
          :robot-seq="props.robot.seq" 
          @lidar-update="handleLidarUpdate"
        />
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue';
import LidarViewer from './LidarViewer.vue'

const props = defineProps({
  robot: Object
});

const lidarPoints = ref(0);
const lastUpdate = ref(null);

const formatTimestamp = (timestamp) => {
  if (!timestamp) return '업데이트 없음';
  const date = new Date(timestamp * 1000);
  return date.toLocaleTimeString();
};

// 라이다 데이터 업데이트 핸들러
const handleLidarUpdate = (points, timestamp) => {
  lidarPoints.value = points;
  lastUpdate.value = timestamp;
};

const getStatusLabel = (status) => {
  const labels = {
    navigating: '이동 중',
    charging: '충전 중',
    emergencyStopped: '정지 중',
    error: '고장',
    waiting: '대기 중',
    homing: '복귀 중',
    patrolling: '순찰 중'
  };
  return labels[status] || status;
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
</script>
