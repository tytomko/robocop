<template>
  <div class="w-full h-full p-4 flex flex-col">
    <div class="mb-4">
      <h3 class="text-lg font-semibold">
        {{ props.cameraType === 'front' ? '전방' : '후방' }} 카메라
      </h3>
    </div>
    <div class="flex-1 min-h-0 bg-black rounded-lg overflow-hidden flex items-center justify-center">
      <img 
        :src="`https://robocopbackendssafy.duckdns.org/api/v1/cameras/video_feed`" 
        alt="카메라 스트림"
        class="w-full h-full object-contain transform translate-z-0 backface-hidden perspective-1000 will-change-transform"
      />
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  robotSeq: {
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
