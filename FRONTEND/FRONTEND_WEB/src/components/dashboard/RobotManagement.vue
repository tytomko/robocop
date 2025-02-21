<template>
  <Teleport to="body">
    <div v-if="show">
      <!-- Modal Overlay -->
      <div class="modal-overlay"></div>
      
      <!-- Modal Container -->
      <div class="modal-container">
        <div class="modal-content p-6">
          <div class="flex justify-between items-center mb-4">
            <h3 class="text-lg font-semibold">로봇 관리</h3>
            <div class="flex space-x-2">
              <button 
                @click="$emit('openAddRobotModal')" 
                class="bg-green-500 text-white px-4 py-2 rounded hover:bg-green-600"
              >
                로봇 등록
              </button>
              <button 
                @click="$emit('close')" 
                class="bg-red-500 text-white px-4 py-2 rounded hover:bg-red-600"
              >
                닫기
              </button>
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
                <tr v-for="robot in robots" :key="robot.seq" class="border-b border-gray-200 hover:bg-gray-50 text-center">
                  <td class="table-data-cell">{{ robot.seq }}</td>
                  <td class="table-data-cell">{{ robot.nickname || robot.name }}</td>
                  <td class="table-data-cell">{{ robot.ipAddress }}</td>
                  <td class="table-data-cell">{{ robot.battery.level }}%</td>
                  <td class="table-data-cell">{{ robot.position ? `x: ${formatCoordinate(robot.position.x)}, y: ${formatCoordinate(robot.position.y)}` : '알 수 없음' }}</td>
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
    </div>
  </Teleport>
</template>

<script setup>
import { Teleport } from 'vue'
import { useRobotCommandsStore } from '@/stores/robotCommands'

const props = defineProps({
  show: Boolean,
  robots: Array
});

const emit = defineEmits(['close', 'openAddRobotModal']);
const robotCommandsStore = useRobotCommandsStore();

const handleBreakdown = async (robotSeq) => {
  try {
    const newStatus = await robotCommandsStore.robotBreakdownCommand(robotSeq);
    const robot = props.robots.find(r => r.seq === robotSeq);
    if (robot && newStatus) {
      robot.status = newStatus;
    }
  } catch (error) {
    console.error('Error in handleBreakdown:', error);
  }
};

const formatCoordinate = (value) => {
  return value ? Number(value).toFixed(2) : '0.00';
};

const handleActivate = async (robotSeq) => {
  try {
    const newStatus = await robotCommandsStore.robotActivateCommand(robotSeq);
    const robot = props.robots.find(r => r.seq === robotSeq);
    if (robot && newStatus) {
      robot.status = newStatus;
    }
  } catch (error) {
    console.error('Error in handleActivate:', error);
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