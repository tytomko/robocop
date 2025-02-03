<template>
    <div class="robot-detail">
      <h1 v-if="robot">{{ robot.nickname || robot.name }}</h1>
      <p v-else>로봇 정보를 불러오는 중...</p>
        
      <!-- 기본 정보 -->
      <div v-if="robot" class="robot-info">
        <p><strong>배터리:</strong> {{ robot.battery }}%</p>
        <p><strong>현재 상태:</strong> {{ getStatusLabel(robot.status) }}</p>
        <p><strong>위치:</strong> {{ robot.location }}</p>
        <p><strong>IP 주소:</strong> {{ robot.ipAddress }}</p>
        <div class="status-item">
            <p class="label"><strong>센서 상태:</strong></p>
            <div class="sensor-status">
            <span v-for="(status, sensor) in robot.sensors" 
                    :key="sensor"
                    class="sensor-badge"
                    :class="status">
                {{ getSensorLabel(sensor) }}
            </span>
            </div>
        </div>
      </div>
            
      <!-- 수동 제어 파트-->
      <div class="manual-control">
        <h3>수동 제어</h3>
        <p>화살표</p>
      </div>

      <button class="back-btn" @click="goBack">뒤로 가기</button>
    </div>
  </template>
  
  <script setup>
  import { ref, computed, onMounted, watch } from 'vue';
  import { useRoute, useRouter } from 'vue-router';
  import { useRobotsStore } from '@/stores/robots';
  import Cctv from '@/components/dashboard/Cctv.vue'

  const route = useRoute();
  const router = useRouter();
  const robotsStore = useRobotsStore();
  const robotId = route.params.robotId;
  
  // 로봇 데이터 로드
  const robot = computed(() => {
    return robotsStore.registered_robots.find(r => r.id == robotId) || null;
  });
  
  // 페이지가 로드될 때 로봇 리스트 불러오기
  onMounted(async () => {
    if (!robot.value) {
      await robotsStore.loadRobots(); // API 요청하여 최신 데이터 불러옴
    }
  });
  
  // 데이터가 갱신될 때도 반영
  watch(() => robotsStore.registered_robots, () => {
    if (!robot.value) {
      robotsStore.loadRobots();
    }
  }, { deep: true });
  
  const getStatusLabel = (status) => {
    const labels = {
      active: '활동 중',
      charging: '충전 중',
      stopped: '정지 중',
      error: '오류 발생',
      idle: '대기 중',
      returning: '복귀 중'
    };
    return labels[status] || status;
  };

  // 센서 레이블
const getSensorLabel = (sensor) => {
  const labels = {
    lidar: 'LiDAR',
    location: '위경도',
    velocity: '시속/초속',
    temperature: '섭씨/화씨'
  }
  return labels[sensor] || sensor
}
  
  const goBack = () => {
    router.push('/');
  };
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
  </style>
  