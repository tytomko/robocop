<template>
  <div class="robot-detail">
    <div class="robot-header">
      <h1 v-if="robot">{{ robot.nickname || robot.name }} ({{ robot.is_active ? '활성화' : '비활성화' }})</h1>
      <button @click="openNicknameModal(robot)" class="settings-btn">
        <i class="fas fa-cog"></i>
      </button>
    </div>

    <!-- 로봇 기본 정보 -->
    <div v-if="robot" class="robot-info">
      <div class="status-item">
        <h3 class="info-header">기본 정보</h3>
        <p><strong>상태:</strong> {{ getStatusLabel(robot.status) }}</p>
        <p><strong>배터리:</strong> {{ robot.battery }}%</p>
        <p><strong>네트워크 상태:</strong> {{ robot.network }}</p>
        <p><strong>작동 시작 시간:</strong> {{ robot.starttime }}</p>
      </div>
      <hr>

      <!-- 센서 데이터 -->
      <div class="status-item">
        <h3 class="info-header">센서 데이터</h3>
        <div class="sensor-status">
          <span v-for="(status, sensor) in robot.sensors" 
                :key="sensor"
                class="sensor-badge"
                :class="status">
            {{ getSensorLabel(sensor) }}: {{ status }}
          </span>
        </div>
      </div>
      <hr>
      
      <!-- 라이다 정보 -->
      <div>
        <h3 class="info-header">3D 라이다</h3>
        <div class="lidar-info">
          <p v-if="robot.lidarData">
            <strong>라이다 상태:</strong> {{ robot.lidarData.status }}<br>
            <strong>라이다 거리:</strong> {{ robot.lidarData.distance }} m
          </p>
          <p v-else>라이다 정보가 없습니다.</p>
        </div>
      </div>
    </div>

    <button class="back-btn" @click="goBack">뒤로 가기</button>

    <!-- RobotNickname 모달 컴포넌트 -->
    <RobotNickname 
      v-if="showNicknameModal" 
      :show="showNicknameModal" 
      :robot="selectedRobotForNickname" 
      @close="closeNicknameModal" 
      @save="setRobotNickname" />
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue';
import { useRoute, useRouter } from 'vue-router';
import { useRobotsStore } from '@/stores/robots';
import RobotNickname from '@/components/detail/RobotNickname.vue';

const route = useRoute();
const router = useRouter();
const robotsStore = useRobotsStore();
const robotId = route.params.robotId;

const robot = computed(() => {
  return robotsStore.registered_robots.find(r => r.id == robotId) || null;
});

const getStatusLabel = (status) => {
  const labels = {
    active: '활동 중',
    charging: '충전 중',
    stopped: '정지 중',
    error: '오류 발생',
    idle: '대기 중',
    returning: '복귀 중',
    breakdown: '고장'
  }
  return labels[status] || status
}

// 닉네임 모달 상태
const showNicknameModal = ref(false);
const selectedRobotForNickname = ref(null);

// 로봇 닉네임 모달 열기
const openNicknameModal = (robot) => {
  selectedRobotForNickname.value = { id: robot.id, nickname: robot.nickname || '' };
  showNicknameModal.value = true;
};

// 닉네임 모달 닫기
const closeNicknameModal = () => {
  showNicknameModal.value = false;
};

// 닉네임 저장
const setRobotNickname = (robotId, nickname) => {
  localStorage.setItem(`robot_nickname_${robotId}`, nickname);

  // robots 상태 업데이트
  const robotIndex = robotsStore.registered_robots.findIndex(r => r.id === robotId);
  if (robotIndex !== -1) {
    robotsStore.registered_robots[robotIndex].nickname = nickname;
  }

  showNicknameModal.value = false;
};

const goBack = () => {
  router.push('/');
};

onMounted(() => {
  robotsStore.loadRobots();
});

watch(() => robotsStore.registered_robots, () => {
  if (!robot.value) {
    robotsStore.loadRobots();
  }
}, { deep: true });
</script>

<style scoped>
.robot-detail {
  padding: 20px;
  max-width: 600px;
  margin: 0 auto;
  overflow-y: auto; /* 세로 스크롤 활성화 */
  max-height: 80vh; /* 화면 높이를 넘어가지 않게 제한 */
}

.robot-header {
  display: flex;
  align-items: center; /* 세로 중앙 정렬 */
  gap: 10px; /* h1과 버튼 사이 여백 */
}

.robot-info {
  background: #f8f9fa;
  padding: 15px;
  border-radius: 8px;
}

.back-btn {
  display: block;
  margin: 20px auto;
  padding: 10px 15px;
  background: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.back-btn:hover {
  background: #0056b3;
}

.sensor-status {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
}

.sensor-badge {
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
}

.sensor-badge.normal {
  background: #28a745;
  color: white;
}

.sensor-badge.warning {
  background: #ffc107;
  color: #000;
}

.sensor-badge.error {
  background: #dc3545;
  color: white;
}

.lidar-info {
  margin-top: 10px;
}

.settings-btn {
  background: none;
  border: none;
  cursor: pointer;
  font-size: 1.2rem;
}

.settings-btn i {
  color: #333; /* 아이콘 색상 */
}

</style>
