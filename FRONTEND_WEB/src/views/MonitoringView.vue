<template>
  <div class="h-full flex flex-col gap-1.5 p-5 overflow-y-auto bg-gray-100">
    <div class="border-b pb-2 mb-4">
      <h2 class="text-2xl font-bold text-gray-800">실시간 모니터링</h2>
    </div>

    <!-- 탭 메뉴 -->
    <div class="flex border-b">
      <button
        v-for="tab in tabs"
        :key="tab.name"
        @click="handleTabChange(tab.name)"
        class="px-6 py-3 text-gray-600 font-semibold transition-all"
        :class="{
          'border-b-4 border-blue-500 text-blue-600': activeTab === tab.name,
          'hover:text-blue-500': activeTab !== tab.name
        }"
      >
        {{ tab.label }}
      </button>
    </div>

    <!-- 탭 콘텐츠 -->
    <div class="mt-4">
      <!-- RobotMap 탭 -->
      <div v-if="activeTab === 'robotMap'" class="bg-white rounded-lg shadow-md p-5 font-sans">
        <RobotMap 
          v-if="isMapReady"
          :key="mapKey"
          :showSelectedNodes="false" 
          :isMonitoringMode="true" 
          ref="robotMapRef"
        />
        <div v-else class="flex justify-center items-center h-64">
          <div class="text-gray-500">로딩 중...</div>
        </div>
      </div>
      
      <!-- 다른 탭들 -->
      <RobotList v-if="activeTab === 'robotList'" />
      <StatisticsView v-if="activeTab === 'statistics'" />
    </div>

    <!-- 모달들 -->
    <RobotManagement
      :show="robotsStore.showRobotManagementModal"
      :robots="robots"
      @close="robotsStore.closeRobotManagementModal"
      @openAddRobotModal="robotsStore.openAddRobotModal"
    />

    <RobotRegistration
      :show="robotsStore.showModal"
      :newRobot="robotsStore.newRobot"
      @handleAddRobot="robotsStore.handleAddRobot"
      @close="robotsStore.closeModal"
    />
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted, watch } from 'vue';
import { useRobotsStore } from '@/stores/robots';
import RobotList from '@/components/dashboard/RobotList.vue';
import RobotManagement from '@/components/dashboard/RobotManagement.vue';
import RobotRegistration from '@/components/dashboard/RobotRegistration.vue';
import RobotMap from '@/components/map/RobotMap.vue';
import StatisticsView from '@/views/statistics/StatisticsView.vue';

const robotsStore = useRobotsStore();
const robots = computed(() => robotsStore.robots);
const robotMapRef = ref(null);
const isMapReady = ref(false);
const mapKey = ref(Date.now());

// 탭 관리
const activeTab = ref('robotMap');
const tabs = ref([
  { name: 'robotMap', label: '실시간 로봇 위치' },
  { name: 'robotList', label: '로봇 목록' },
  { name: 'statistics', label: '통계' }
]);

// 로봇 데이터 로드 함수
async function loadRobotData() {
  try {
    await robotsStore.loadRobots();
    isMapReady.value = true;
  } catch (error) {
    console.error('로봇 데이터 로드 실패:', error);
  }
}

// 탭 변경 핸들러
async function handleTabChange(tabName) {
  activeTab.value = tabName;
  if (tabName === 'robotMap') {
    // 맵 탭으로 돌아올 때 맵 리프레시
    mapKey.value = Date.now();
    if (!isMapReady.value) {
      await loadRobotData();
    }
  }
}

// 맵 초기화 감시
watch(robots, (newRobots) => {
  if (newRobots.length > 0 && !isMapReady.value) {
    isMapReady.value = true;
  }
}, { immediate: true });

onMounted(async () => {
  // 컴포넌트 마운트 시 로봇 데이터 로드
  await loadRobotData();
});

onUnmounted(() => {
  // 필요한 정리 작업이 있다면 여기서 수행
  isMapReady.value = false;
});
</script>