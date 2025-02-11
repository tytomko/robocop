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

  // 로봇 위치를 아이콘으로 표시
  L.marker([52.505, -0.09]).addTo(map.value)
    .bindPopup('현재 로봇 위치')
    .openPopup();
});
</script>

<style scoped>
/* #map 컨테이너의 z-index를 낮춥니다. (원하는 값으로 조정) */
#map {
  position: relative;
  z-index: 0;
}

/* 만약 Leaflet의 내부 pane 들이 여전히 위에 표시된다면 아래와 같이 override 해볼 수 있습니다.
   (다른 요소와의 stacking 순서를 고려하여 적절한 값으로 조정하세요.) */
.leaflet-pane {
  z-index: 0 !important;
}
</style>
