<template>
  <div class="bg-gray-100 p-4 rounded-lg shadow">
    <!-- 기본 정보 -->
    <div class="mb-4">
      <h3 class="text-lg font-semibold mb-2">기본 정보</h3>
      <p><strong>상태:</strong> {{ getStatusLabel(robot.status) }}</p>
      <p><strong>배터리:</strong> {{ robot.battery }}%</p>
      <p><strong>네트워크 상태:</strong> {{ robot.networkHealth }} ({{ robot.networkStatus }})</p>
      <p><strong>CPU 온도:</strong> {{ robot.cpuTemp }}°C</p>
      <p><strong>속도: {{ robot.motion }}</strong></p>
      <p><strong>작동 시작 시간:</strong> {{ getOperationTime(robot.startAt) }}</p>
    </div>
    <hr class="border-gray-300 my-4">


    <!-- 센서 데이터 -->
    <div class="mb-4">
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
    <hr class="border-gray-300 my-4">

    <!-- 3D 라이다 정보 -->
    <div>
      <h3 class="text-lg font-semibold mb-2">3D 라이다</h3>
      <div>
        <p v-if="robot.lidarData">
          <strong>라이다 상태:</strong> {{ robot.lidarData.status }}<br>
          <strong>라이다 거리:</strong> {{ robot.lidarData.distance }} m
        </p>
        <p v-else class="text-gray-500">라이다 정보가 없습니다.</p>
      </div>
    </div>
  </div>
</template>

<script setup>
defineProps({
  robot: Object
});

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

const getSensorClass = (status) => {
  return {
    normal: 'bg-green-500',
    warning: 'bg-yellow-500 text-black',
    error: 'bg-red-500'
  }[status] || 'bg-gray-500';
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
