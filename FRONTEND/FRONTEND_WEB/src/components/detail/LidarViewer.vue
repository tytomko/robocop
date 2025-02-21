<template>
  <div ref="container" class="w-full h-[400px] bg-black rounded-lg">
    <!-- Three.js 렌더링 컨테이너 -->
    <!-- 숨겨진 버튼 - 개발자 도구에서만 보임 -->
    <button
      class="absolute opacity-0 cursor-default"
      style="top: -9999px; left: -9999px;"
      @click="connectToLidarSSE"
    >
      라이다 실시간 데이터 연결
    </button>
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
let isLiveDataEnabled = false;  // 실시간 데이터 활성화 상태

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

// SSE 연결 함수를 별도로 분리
const connectToLidarSSE = () => {
  if (isLiveDataEnabled) return;  // 이미 연결된 경우 중복 연결 방지
  
  isLiveDataEnabled = true;
  eventSource = new EventSource(
    `https://robocopbackendssafy.duckdns.org/api/v1/lidar/sse/${props.robotSeq}`
  );
  
  eventSource.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.pcd) {
      updatePointCloud(data.pcd);
      emit('lidar-update', data.pcd.points, data.timestamp);
    }
  };
  
  eventSource.onerror = (error) => {
    console.error('라이다 SSE 연결 에러:', error);
    if (!isSSEFailed) {
      isSSEFailed = true;
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

// 컴포넌트 마운트 시
onMounted(() => {
  initThree();
  loadStaticPCD();  // 정적 PCD 파일만 로드
  window.addEventListener('resize', handleResize);
});

// 컴포넌트 언마운트 시
onUnmounted(() => {
  if (eventSource) {
    eventSource.close();
    isLiveDataEnabled = false;  // 상태 초기화
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

// robotSeq가 변경될 때
watch(() => props.robotSeq, (newSeq) => {
  isSSEFailed = false;
  loadStaticPCD();  // 정적 PCD 파일만 로드
});
</script>

<style scoped>
.point-cloud-container {
  position: relative;
  width: 100%;
  height: 400px;
}
</style> 