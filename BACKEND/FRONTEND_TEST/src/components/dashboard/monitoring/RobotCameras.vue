<template>
  <div class="cameras-container">
    <div class="camera-wrapper">
      <h3>전방 카메라</h3>
      <CameraView 
        robotId="robot1"
        cameraType="front"
        :rosHost="rosHost"
        :rosPort="rosPort"
      />
    </div>
    <div class="camera-wrapper">
      <h3>후방 카메라</h3>
      <CameraView 
        robotId="robot1"
        cameraType="rear"
        :rosHost="rosHost"
        :rosPort="rosPort"
      />
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue';
import CameraView from './CameraView.vue';

const rosHost = ref('172.30.1.78');
const rosPort = ref(9090);
</script>

<style scoped>
.cameras-container {
  display: flex;
  flex-direction: column;  /* 위아래로 배치 */
  gap: 20px;
  width: 100%;  /* 전체 너비 사용 */
  max-width: 1200px;  /* 최대 너비 설정 */
  margin: 0 auto;  /* 중앙 정렬 */
}

.camera-wrapper {
  width: 100%;  /* 전체 너비 사용 */
  background: #f5f5f5;
  border-radius: 8px;
  padding: 15px;
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

/* 카메라 뷰의 비율을 16:9로 유지 */
.camera-wrapper :deep(.camera-container) {
  position: relative;
  width: 100%;
  padding-top: 56.25%;  /* 16:9 비율 */
  background: #000;
}

.camera-wrapper :deep(.camera-container video) {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  object-fit: contain;
}

/* 반응형 디자인 */
@media (max-width: 768px) {
  .cameras-container {
    padding: 10px;
  }
  
  .camera-wrapper {
    padding: 10px;
  }
}
</style> 