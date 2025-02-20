<template>
  <div ref="container" class="w-full h-[400px] bg-black rounded-lg">
    <!-- Three.js 렌더링 컨테이너 -->
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { PCDLoader } from 'three/examples/jsm/loaders/PCDLoader';

const props = defineProps({
  robotSeq: {
    type: Number,
    required: true
  }
});

const emit = defineEmits(['lidar-update']);

const container = ref(null);
let scene, camera, renderer, controls;
let pointCloud;
let eventSource = null;
let isSSEFailed = false;

// Three.js 초기화
const initThree = () => {
  // Scene 설정
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x000000);

  // Camera 설정
  camera = new THREE.PerspectiveCamera(
    75,
    container.value.clientWidth / container.value.clientHeight,
    0.1,
    1000
  );
  camera.position.set(5, 5, 5);
  camera.lookAt(0, 0, 0);

  // Renderer 설정
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.value.clientWidth, container.value.clientHeight);
  container.value.appendChild(renderer.domElement);

  // Controls 설정
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;

  // 좌표축 헬퍼 추가
  const axesHelper = new THREE.AxesHelper(5);
  scene.add(axesHelper);

  // 초기 포인트 클라우드 생성
  const geometry = new THREE.BufferGeometry();
  const material = new THREE.PointsMaterial({
    size: 0.05,
    vertexColors: true
  });
  pointCloud = new THREE.Points(geometry, material);
  scene.add(pointCloud);

  // 애니메이션 루프 시작
  animate();
};

// 포인트 클라우드 업데이트
const updatePointCloud = (pcdData) => {
  if (!pcdData || !pcdData.positions || pcdData.positions.length === 0) return;

  const positions = new Float32Array(pcdData.positions);
  const colors = new Float32Array(pcdData.positions.length);

  // 강도값을 기반으로 색상 설정
  for (let i = 0; i < pcdData.intensities.length; i++) {
    const intensity = pcdData.intensities[i];
    const colorIndex = i * 3;
    colors[colorIndex] = 1.0;     // R (흰색)
    colors[colorIndex + 1] = 1.0; // G (흰색)
    colors[colorIndex + 2] = 1.0; // B (흰색)
  }

  pointCloud.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  pointCloud.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  pointCloud.geometry.computeBoundingSphere();
};

// 정적 PCD 파일 로드
const loadStaticPCD = () => {
  const loader = new PCDLoader();
  const pcdUrl = 'https://robocopbackendssafy.duckdns.org/media/persons_image/map.pcd';
  
  loader.load(
    pcdUrl,
    (points) => {
      if (pointCloud) {
        scene.remove(pointCloud);
      }
      pointCloud = points;
      scene.add(pointCloud);
      
      // 포인트 크기 조정
      pointCloud.material.size = 0.05;
      
      // 뷰 중앙으로 조정
      const box = new THREE.Box3().setFromObject(pointCloud);
      const center = box.getCenter(new THREE.Vector3());
      pointCloud.position.sub(center);
    },
    (xhr) => {
      console.log((xhr.loaded / xhr.total * 100) + '% loaded');
    },
    (error) => {
      console.error('PCD 파일 로드 실패:', error);
    }
  );
};

// SSE 연결 설정
const connectToLidarSSE = (robotSeq) => {
  if (eventSource) {
    eventSource.close();
  }

  eventSource = new EventSource(`https://robocopbackendssafy.duckdns.org/api/v1/lidar/sse/${robotSeq}`);
  eventSource.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.pcd) {
      updatePointCloud(data.pcd);
      // 부모 컴포넌트에 데이터 전달
      emit('lidar-update', data.pcd.points, data.timestamp);
    }
  };

  eventSource.onerror = (error) => {
    console.error('라이다 SSE 연결 에러:', error);
    eventSource.close();
    if (!isSSEFailed) {
      isSSEFailed = true;
      console.log('SSE 연결 실패, 정적 PCD 파일 로드 시도');
      loadStaticPCD();
    }
  };
};

// 애니메이션 루프
const animate = () => {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
};

// 윈도우 리사이즈 핸들러
const handleResize = () => {
  if (!container.value) return;
  
  const width = container.value.clientWidth;
  const height = container.value.clientHeight;

  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
};

// 컴포넌트 마운트 시 초기화
onMounted(() => {
  initThree();
  connectToLidarSSE(props.robotSeq);
  window.addEventListener('resize', handleResize);
});

// 컴포넌트 언마운트 시 정리
onUnmounted(() => {
  if (eventSource) {
    eventSource.close();
  }
  window.removeEventListener('resize', handleResize);
  if (renderer) {
    renderer.dispose();
  }
  if (container.value) {
    container.value.innerHTML = '';
  }
  isSSEFailed = false;
});

// robotSeq가 변경될 때 SSE 재연결
watch(() => props.robotSeq, (newSeq) => {
  isSSEFailed = false;
  connectToLidarSSE(newSeq);
});
</script>

<style scoped>
.point-cloud-container {
  position: relative;
  width: 100%;
  height: 400px;
}
</style> 