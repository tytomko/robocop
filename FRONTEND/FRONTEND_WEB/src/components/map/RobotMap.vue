<template>
  <div id="map" class="h-[500px] relative z-0"></div>
</template>

<script setup>
import { onMounted, ref } from 'vue';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

const map = ref(null);

onMounted(() => {
  map.value = L.map('map').setView([51.505, -0.09], 13);  // 기본 위치 설정
  
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
  }).addTo(map.value);

  // 로봇 위치를 아이콘으로 표시
  L.marker([52.505, -0.09]).addTo(map.value)
    .bindPopup('현재 로봇 위치')
    .openPopup();
});
</script>

<style scoped>
/***** Leaflet 기본 스타일 유지 *****/
.leaflet-pane {
  z-index: 0 !important;
}
</style>
