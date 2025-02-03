<template>
  <div id="map" style="height: 500px;"></div>
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

  // 로봇 위치를 아이콘으로 표시할 수 있습니다.
  L.marker([51.505, -0.09]).addTo(map.value)
    .bindPopup('현재 로봇 위치')
    .openPopup();
});
</script>