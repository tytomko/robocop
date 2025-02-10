<template>
  <div v-if="show" class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
    <div class="bg-white p-6 rounded-lg w-3/4 max-w-2xl">
      <div class="flex justify-between items-center mb-4">
        <h3 class="text-lg font-semibold">로봇 관리</h3>
        <div class="flex space-x-2">
          <button @click="$emit('openAddRobotModal')" class="bg-green-500 text-white px-4 py-2 rounded hover:bg-green-600">로봇 등록</button>
          <button @click="$emit('close')" class="bg-red-500 text-white px-4 py-2 rounded hover:bg-red-600">닫기</button>
        </div>
      </div>

      <table class="w-full border-collapse border border-gray-200 text-sm text-center">
        <thead>
          <tr class="bg-gray-100">
            <th class="border border-gray-200 px-4 py-2">ID</th>
            <th class="border border-gray-200 px-4 py-2">이름</th>
            <th class="border border-gray-200 px-4 py-2">IP 주소</th>
            <th class="border border-gray-200 px-4 py-2">배터리</th>
            <th class="border border-gray-200 px-4 py-2">위치</th>
            <th class="border border-gray-200 px-4 py-2">상태</th>
            <th class="border border-gray-200 px-4 py-2">작업</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="robot in robots" :key="robot.id" class="border border-gray-200">
            <td class="px-4 py-2">{{ robot.id }}</td>
            <td class="px-4 py-2">{{ robot.nickname || robot.name }}</td>
            <td class="px-4 py-2">{{ robot.ipAddress }}</td>
            <td class="px-4 py-2">{{ robot.battery }}%</td>
            <td class="px-4 py-2">{{ robot.location }}</td>
            <td class="px-4 py-2">
              <span :class="getStatusClass(robot.status)" class="px-2 py-1 rounded text-white">{{ getStatusLabel(robot.status) }}</span>
            </td>
            <td class="px-4 py-2">
              <button v-if="robot.status !== 'breakdown'" class="bg-black text-white px-3 py-1 rounded hover:bg-gray-600" @click="setBreakdown(robot.id)">고장</button>
              <button v-else class="bg-green-500 text-white px-3 py-1 rounded hover:bg-green-600" @click="setActive(robot.id)">가동</button>
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  show: Boolean,
  robots: Array
});

const emit = defineEmits(['close', 'openAddRobotModal', 'setBreakdown', 'setActive']);

const getStatusClass = (status) => {
  const statusClasses = {
    active: 'bg-green-500',
    charging: 'bg-blue-500',
    stopped: 'bg-red-500',
    error: 'bg-red-600',
    idle: 'bg-gray-500',
    returning: 'bg-teal-500',
    breakdown: 'bg-red-500',
    unserviceable: 'bg-black'
  };
  return statusClasses[status] || 'bg-yellow-500';
};

const getStatusLabel = (status) => {
  const labels = {
    active: '활동 중',
    charging: '충전 중',
    stopped: '정지 중',
    error: '오류 발생',
    idle: '대기 중',
    returning: '복귀 중',
    breakdown: '고장',
    unserviceable: '사용 불가'
  };
  return labels[status] || status;
};

const setBreakdown = (robotId) => {
  emit('setBreakdown', robotId);
};

const setActive = (robotId) => {
  emit('setActive', robotId);
};
</script>
