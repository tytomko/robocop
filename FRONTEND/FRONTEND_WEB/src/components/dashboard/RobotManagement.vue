<template>
  <div v-if="show" class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
    <div class="bg-white p-6 rounded-lg w-full max-w-4xl"> <!-- max-w-[600px]에서 max-w-4xl로 변경 -->
      <div class="flex justify-between items-center mb-4">
        <h3 class="text-lg font-semibold">로봇 관리</h3>
        <div class="flex space-x-2">
          <button @click="$emit('openAddRobotModal')" class="bg-green-500 text-white px-4 py-2 rounded hover:bg-green-600">로봇 등록</button>
          <button @click="$emit('close')" class="bg-red-500 text-white px-4 py-2 rounded hover:bg-red-600">닫기</button>
        </div>
      </div>

      <div class="overflow-x-auto max-h-[240px] rounded-lg border border-gray-200">
        <table class="min-w-full table-auto">
          <thead>
            <tr class="bg-gray-100">
              <th class="table-header-cell w-16">ID</th>
              <th class="table-header-cell w-32">이름</th>
              <th class="table-header-cell w-40">IP 주소</th>
              <th class="table-header-cell w-24">배터리</th>
              <th class="table-header-cell w-32">위치</th>
              <th class="table-header-cell w-24">상태</th>
              <th class="table-header-cell w-24">작업</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="robot in robots" :key="robot.seq" class="border-b border-gray-200 hover:bg-gray-50">
              <td class="table-data-cell">{{ robot.seq }}</td>
              <td class="table-data-cell">{{ robot.nickname || robot.name }}</td>
              <td class="table-data-cell">{{ robot.ipAddress }}</td>
              <td class="table-data-cell">{{ robot.battery }}%</td>
              <td class="table-data-cell">{{ robot.position }}</td>
              <td class="table-data-cell">
                <span :class="['status-badge', getStatusClass(robot.status)]">
                  {{ getStatusLabel(robot.status) }}
                </span>
              </td>
              <td class="table-data-cell">
                <button 
                  v-if="robot.status !== 'error'" 
                  class="manage-btn-error" 
                  @click="handleBreakdown(robot.seq)"
                >고장</button>
                <button 
                  v-else 
                  class="manage-btn-activate" 
                  @click="handleActivate(robot.seq)"
                >가동</button>
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  show: Boolean,
  robots: Array
});

const emit = defineEmits(['close', 'openAddRobotModal']);

const handleBreakdown = (robotSeq) => {
  const robot = props.robots.find(r => r.seq === robotSeq);
  if (robot) {
    robot.status = 'error';
  }
};

const handleActivate = (robotSeq) => {
  const robot = props.robots.find(r => r.seq === robotSeq);
  if (robot) {
    robot.status = 'navigating';
  }
};

const getStatusClass = (status) => {
  const statusClasses = {
    charging: 'bg-green-500',
    navigating: 'bg-blue-500',
    patrolling: 'bg-blue-500',
    emergencyStopped: 'bg-red-500',
    error: 'bg-red-600',
    waiting: 'bg-gray-500',
    homing: 'bg-teal-500',
  };
  return statusClasses[status] || 'bg-yellow-500';
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
</script>