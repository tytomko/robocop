<template>
  <div v-if="show" class="modal-overlay">
    <div class="modal-container management">
      <div class="modal-header">
        <h3>로봇 관리</h3>
        <div class="modal-buttons">
          <button @click="$emit('openAddRobotModal')" class="add-robot-btn">로봇 등록</button>
          <button @click="$emit('close')" class="close-btn">닫기</button>
        </div>
      </div>

      <table class="robot-table">
        <thead>
          <tr>
            <th>ID</th>
            <th>이름</th>
            <th>IP 주소</th>
            <th>배터리</th>
            <th>위치</th>
            <th>상태</th>
            <th>작업</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="robot in robots" :key="robot.id">
            <td>{{ robot.id }}</td>
            <td>{{ robot.nickname || robot.name }}</td>
            <td>{{ robot.ipAddress }}</td>
            <td>{{ robot.battery }}%</td>
            <td>{{ robot.location }}</td>
            <td>
              <span :class="getStatusClass(robot.status)">{{ getStatusLabel(robot.status) }}</span>
            </td>
            <td>
              <button v-if="robot.status !== 'breakdown'" class="action-btn breakdown" @click="setBreakdown(robot.id)">고장</button>
              <button v-else class="action-btn active" @click="setActive(robot.id)">가동</button>
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
  return `status-badge ${status}`;
};

const getStatusLabel = (status) => {
  const labels = {
    active: '활동 중',
    charging: '충전 중',
    stopped: '정지 중',
    error: '오류 발생',
    idle: '대기 중',
    returning: '복귀 중',
    breakdown: '고장'
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

<style scoped>
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 999;
}

.modal-container {
  background: white;
  padding: 20px;
  border-radius: 10px;
  width: 600px;
}

.status-badge {
  padding: 5px 10px;
  border-radius: 999px;
  font-size: 0.875rem;
  font-weight: 500;
  display: inline-block;
}

.status-badge.active {
  background-color: #28a745;
  color: white;
}

.status-badge.charging {
  background-color: #007bff;
  color: white;
}

.status-badge.stopped {
  background-color: #dc3545;
  color: white;
}

.status-badge.error {
  background-color: #dc3545;
  color: white;
}

.status-badge.idle {
  background-color: #6c757d;
  color: white;
}

.status-badge.returning {
  background-color: black;
  color: white;
}

.status-badge.unknown {
  background-color: #ffc107;
  color: black;
}

.status-badge.breakdown {
  background-color: black;
  color: white;
}

.status-details {
  display: flex;
  flex-direction: column;
  gap: 15px;
  margin: 15px 0;
}

.status-item {
  margin-bottom: 0.5rem;
}

.status-item .label {
  display: block;
  color: #666;
  font-size: 0.875rem;
  margin-bottom: 0.25rem;
}

.action-btn.breakdown {
  background: #dc3545;
  color: white;
}

.action-btn.breakdown:hover {
  background: #c82333;
}

.action-btn {
  flex: 1;
  padding: 8px;
  border: none;
  border-radius: 4px;
  background: #007bff;
  color: white;
  cursor: pointer;
  transition: background-color 0.3s;
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 15px;
}

.modal-buttons {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
}

.modal-buttons button {
  padding: 10px 15px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-size: 14px;
}

.modal-buttons .close-btn {
  background: #dc3545;
  color: white;
}

.modal-buttons .close-btn:hover {
  background: #c82333;
}

.add-robot-btn {
  background: #28a745;
  color: white;
  padding: 5px 10px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
}

/* 테이블 스타일 */
.robot-table {
  width: 100%;
  border-collapse: collapse;
}

.robot-table th, .robot-table td {
  border: 1px solid #ddd;
  padding: 10px;
  text-align: center;
}

.robot-table th {
  background: #f4f4f4;
}
</style>
