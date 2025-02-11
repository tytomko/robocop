<template>
  <div class="cameras-container">
    <!-- DB에서 가져오는 대신 하드코딩된 로봇 -->
    <div class="robot-cameras">
      <h2 class="robot-title">ssafy</h2>
      <div class="camera-wrapper">
        <h3>전방 카메라</h3>
        <CameraView 
          robotId="ssafy"
          cameraType="front"
        />
      </div>
      <div class="camera-wrapper">
        <h3>후방 카메라</h3>
        <CameraView 
          robotId="ssafy"
          cameraType="rear"
        />
      </div>
    </div>
  </div>
</template>

<script setup>
import CameraView from './CameraView.vue';

// DB 관련 코드 모두 주석 처리
// import { ref, onMounted } from 'vue';
// const robots = ref([]);
// const loading = ref(true);
// const error = ref(null);

// async function fetchRobots() {
//   try {
//     const response = await fetch('/api/v1/robots', {
//       credentials: 'include'
//     });
    
//     if (!response.ok) {
//       throw new Error('로봇 정보를 불러오는데 실패했습니다.');
//     }

//     const result = await response.json();
//     if (!result.success) {
//       throw new Error(result.message || '로봇 정보를 불러오는데 실패했습니다.');
//     }

//     robots.value = result.data.filter(robot => robot.isActive && !robot.isDeleted);
    
//     if (robots.value.length === 0) {
//       error.value = '연결된 로봇이 없습니다.';
//     }
//   } catch (e) {
//     console.error('로봇 정보 조회 실패:', e);
//     error.value = e.message;
//   } finally {
//     loading.value = false;
//   }
// }

// onMounted(() => {
//   fetchRobots();
// });
</script>

<style scoped>
.cameras-container {
  padding: 20px;
  max-width: 1400px;
  margin: 0 auto;
}

.robot-cameras {
  margin-bottom: 40px;
}

.robot-title {
  font-size: 1.5em;
  color: #333;
  margin-bottom: 20px;
  padding: 10px;
  background: #f8f9fa;
  border-radius: 4px;
  border-left: 4px solid #007bff;
}

.camera-wrapper {
  margin-bottom: 20px;
}

.camera-wrapper h3 {
  margin-bottom: 10px;
  color: #333;
  font-size: 1.1em;
  padding: 8px;
  background: #2a2a2a;
  color: white;
  border-radius: 4px;
}

.loading, .error {
  text-align: center;
  padding: 20px;
  font-size: 1.1em;
  color: #666;
}

.error {
  color: #dc3545;
}

@media (max-width: 768px) {
  .cameras-container {
    padding: 10px;
  }
}
</style> 