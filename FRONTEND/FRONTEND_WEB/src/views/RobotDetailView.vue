<template>
  <div class="robot-detail">
    <h1 v-if="robot">{{ robot.nickname || robot.name }} ({{ robot.is_active ? '활성화' : '비활성화' }})</h1>

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
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue';
import { useRoute, useRouter } from 'vue-router';
import { useRobotsStore } from '@/stores/robots';

const route = useRoute();
const router = useRouter();
const robotsStore = useRobotsStore();
const robotId = route.params.robotId;

// 로봇 데이터 로드
const robot = computed(() => {
  return robotsStore.registered_robots.find(r => r.id == robotId) || null;
});

// 상태 레이블
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

// 센서 레이블
const getSensorLabel = (sensor) => {
  const labels = {
    lidar: 'LiDAR',
    location: '위경도',
    velocity: '시속/초속',
    temperature: '섭씨/화씨'
  };
  return labels[sensor] || sensor;
};

const goBack = () => {
  router.push('/');
};

onMounted(() => {
  robotsStore.loadRobots();
});

// 데이터가 갱신될 때도 반영
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
</style>
