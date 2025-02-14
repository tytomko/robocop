<template>
  <div class="flex flex-col items-center justify-center w-full h-vh bg-black text-white">
    <div v-if="!hasPermission" class="text-center">
      <p class="text-lg">카메라 접근 권한이 필요합니다</p>
      <button @click="requestPermission" class="mt-4 px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-700">카메라 권한 요청</button>
    </div>
    <div v-else-if="errorMessage" class="text-center">
      <p class="text-red-500">{{ errorMessage }}</p>
      <button @click="retryConnection" class="mt-4 px-4 py-2 bg-red-500 text-white rounded hover:bg-red-700">다시 시도</button>
    </div>
    <video v-else-if="hasPermission" ref="videoElement" autoplay playsinline class="w-full max-w-2xl border border-gray-700 rounded-lg"></video>
    <div v-else class="text-lg">카메라 연결 중...</div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted } from 'vue';

const ws = ref(null);
const videoElement = ref(null);

const errorMessage = ref('');
const hasPermission = ref(false);
const stream = ref(null);
const streamInfo = ref(null);

const requestPermission = async () => {
  try {
    if (!navigator.mediaDevices) {
      throw new Error('이 브라우저는 카메라 접근을 지원하지 않습니다.');
    }

    if (stream.value) {
      stream.value.getTracks().forEach(track => track.stop());
    }

    stream.value = await navigator.mediaDevices.getUserMedia({
      video: { width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30 } }
    });
    
    if (!stream.value) {
      throw new Error('카메라 스트림을 가져올 수 없습니다.');
    }

    if (videoElement.value) {
      videoElement.value.srcObject = stream.value;
      hasPermission.value = true;
      errorMessage.value = '';
    }

    const videoTrack = stream.value.getVideoTracks()[0];
    if (videoTrack) {
      const settings = videoTrack.getSettings();
      streamInfo.value = { width: settings.width || 640, height: settings.height || 480 };
    }
  } catch (error) {
    console.error('카메라 권한 에러:', error);
    hasPermission.value = false;
    errorMessage.value = error.name === 'NotAllowedError' 
      ? '카메라 접근이 거부되었습니다. 브라우저 설정에서 카메라 권한을 허용해주세요.'
      : '카메라 연결 중 오류가 발생했습니다.';
  }
};

onMounted(async () => {
  try {
    await requestPermission();
  } catch (error) {
    console.error('카메라 초기화 실패:', error);
  }
});

onUnmounted(() => {
  if (stream.value) {
    stream.value.getTracks().forEach(track => track.stop());
  }
});
</script>
