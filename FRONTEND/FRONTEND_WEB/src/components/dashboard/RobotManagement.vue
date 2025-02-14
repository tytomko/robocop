<template>
  <div v-if="show" class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
    <div class="bg-white p-6 rounded-lg w-full max-w-xl"> <!-- 모달 크기 조정 -->
      <div class="flex justify-between items-center mb-4">
        <h3 class="text-lg font-semibold">로봇 관리</h3>
        <div class="flex space-x-2">
          <button @click="$emit('openAddRobotModal')" class="bg-green-500 text-white px-4 py-2 rounded hover:bg-green-600">로봇 등록</button>
          <button @click="$emit('close')" class="bg-red-500 text-white px-4 py-2 rounded hover:bg-red-600">닫기</button>
        </div>
      </div>

      <!-- 테이블을 감싸는 div에 스크롤 적용 -->
      <div class="overflow-x-auto max-h-60">
        <table class="w-full border-collapse border border-gray-200 text-sm text-center">
          <thead>
            <tr class="bg-gray-100">
              <th class="border border-gray-200 px-2 py-2 w-12">ID</th>
              <th class="border border-gray-200 px-2 py-2 w-24">이름</th>
              <th class="border border-gray-200 px-2 py-2 w-32">IP 주소</th>
              <th class="border border-gray-200 px-2 py-2 w-16">배터리</th>
              <th class="border border-gray-200 px-2 py-2 w-24">위치</th>
              <th class="border border-gray-200 px-2 py-2 w-20">상태</th>
              <th class="border border-gray-200 px-2 py-2 w-20">작업</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="robot in robots" :key="robot.seq" class="border border-gray-200">
              <td class="px-2 py-2 truncate">{{ robot.seq }}</td>
              <td class="px-2 py-2 truncate">{{ robot.nickname || robot.name }}</td>
              <td class="px-2 py-2 truncate">{{ robot.ipAddress }}</td>
              <td class="px-2 py-2 truncate">{{ robot.battery }}%</td>
              <td class="px-2 py-2 truncate">{{ robot.position }}</td>
              <td class="px-2 py-2 truncate">
                <span :class="getStatusClass(robot.status)" class="px-2 py-1 rounded text-white">{{ getStatusLabel(robot.status) }}</span>
              </td>
              <td class="px-2 py-2 truncate">
                <button v-if="robot.status !== 'error'" class="bg-black text-white px-3 py-1 rounded hover:bg-gray-600" @click="setBreakdown(robot.seq)">고장</button>
                <button v-else class="bg-green-500 text-white px-3 py-1 rounded hover:bg-green-600" @click="setActive(robot.seq)">가동</button>
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

const emit = defineEmits(['close', 'openAddRobotModal', 'setBreakdown', 'setActive']);

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

const setBreakdown = (robotSeq) => {
  emit('setBreakdown', robotSeq);
};

const setActive = (robotSeq) => {
  emit('setActive', robotSeq);
};
</script>

<style scoped>
/* 모달 크기 제한 */
.max-w-xl {
  max-width: 600px;
}

/* 스크롤 추가 */
.overflow-x-auto {
  overflow-x: auto;
}
.max-h-60 {
  max-height: 240px; /* 최대 높이 설정 (필요하면 조정 가능) */
}

/* 테이블 한 줄 유지 */
th, td {
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
</style>