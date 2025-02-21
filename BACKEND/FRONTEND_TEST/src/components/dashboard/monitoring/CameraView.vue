<template>
  <div class="camera-view">
    <div class="camera-header">
      <h3>{{ props.cameraType === 'front' ? '전방' : '후방' }} 카메라</h3>
    </div>
    <div class="camera-container">
      <img 
        :src="`http://localhost:8000/api/v1/cameras/video_feed`" 
        alt="카메라 스트림"
        :class="{ 'hardware-accelerated': true }"
      />
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  robotId: {
    type: String,
    required: true
  },
  cameraType: {
    type: String,
    required: true,
    validator: (value) => ['front', 'rear'].includes(value)
  }
});
</script>

<style scoped>
.camera-view {
  width: 100%;
  height: 100%;  /* 부모 컨테이너의 전체 높이 사용 */
  padding: 1rem;
  display: flex;
  flex-direction: column;
}

.camera-header {
  margin-bottom: 1rem;
}

.camera-container {
  flex: 1;  /* 남은 공간 모두 사용 */
  min-height: 0;  /* 필요한 경우 축소 허용 */
  background: #000;
  border-radius: 8px;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
}

.hardware-accelerated {
  transform: translateZ(0);
  backface-visibility: hidden;
  perspective: 1000;
  will-change: transform;
}

.camera-container img {
  width: 100%;
  height: 100%;
  object-fit: contain;
  image-rendering: -webkit-optimize-contrast;
}
</style> 