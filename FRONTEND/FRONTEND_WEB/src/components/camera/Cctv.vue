<template>
  <div class="w-full h-full p-4 flex flex-col">
    <div class="mb-1">
      <h3 class="text-lg font-semibold">
        {{ props.cameraType === 'front' ? '전방' : '후방' }} 카메라
      </h3>
    </div>
    <div class="flex-1 min-h-0 bg-black rounded-lg overflow-hidden flex items-center justify-center">
      <img 
        v-if="!error && isValidProps"
        :src="videoUrl" 
        alt="카메라 스트림"
        class="w-full h-full object-contain transform translate-z-0 backface-hidden perspective-1000 will-change-transform"
        @error="handleError"
        @load="isLoading = false"
      />
      <div v-else-if="!isValidProps" class="text-white text-center">
        <p>카메라 정보가 올바르지 않습니다</p>
      </div>
      <div v-else-if="error" class="text-white text-center">
        <p>카메라 스트림을 불러올 수 없습니다</p>
        <button @click="retryLoad" class="mt-2 px-4 py-2 bg-blue-500 rounded-lg">
          다시 시도
        </button>
      </div>
      <div v-if="isLoading" class="text-white">
        로딩 중...
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch } from 'vue';

const props = defineProps({
  robotSeq: {
    type: [String, Number],
    required: true
  },
  cameraType: {
    type: String,
    required: true,
    validator: (value) => ['front', 'rear'].includes(value)
  }
});

// 디버깅을 위한 watch 추가
watch(() => props.robotSeq, (newVal) => {
  console.log('robotSeq:', newVal);
});

watch(() => props.cameraType, (newVal) => {
  console.log('cameraType:', newVal);
});

const error = ref(false);
const isLoading = ref(true);

const isValidProps = computed(() => {
  return props.robotSeq && props.cameraType && 
         props.robotSeq !== 'undefined' && 
         props.cameraType !== 'undefined';
});

const videoUrl = computed(() => {
  if (!isValidProps.value) return '';
  return `https://robocopbackendssafy.duckdns.org/api/v1/cameras/video_feed/${props.robotSeq}/${props.cameraType}`;
});

const handleError = () => {
  error.value = true;
  isLoading.value = false;
};

const retryLoad = () => {
  error.value = false;
  isLoading.value = true;
};
</script>
