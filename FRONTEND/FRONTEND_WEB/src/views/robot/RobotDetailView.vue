<template>
  <div class="robot-detail max-w-xl mx-auto p-5 overflow-y-auto max-h-[80vh]">
    <div class="robot-header flex items-center gap-3">
      <h1 v-if="robot" class="text-xl font-bold">
        {{ robot.nickname || robot.name }} 
        <span class="text-sm" :class="robot.is_active ? 'text-green-500' : 'text-red-500'">
          ({{ robot.is_active ? '활성화' : '비활성화' }})
        </span>
      </h1>
      <button @click="openNicknameModal(robot)" class="text-gray-700 text-lg hover:text-gray-900">
        <i class="fas fa-cog"></i>
      </button>
    </div>

    <!-- RobotInfo 컴포넌트 사용 -->
    <RobotInfo v-if="robot" :robot="robot" />

    <button class="mt-5 w-full py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-700" @click="goBack">
      뒤로 가기
    </button>

    <!-- RobotNickname 모달 컴포넌트 -->
    <RobotNickname 
      v-if="showNicknameModal" 
      :show="showNicknameModal" 
      :robot="selectedRobotForNickname" 
      @close="closeNicknameModal" 
      @save="setRobotNickname" />
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue';
import { useRoute, useRouter } from 'vue-router';
import { useRobotsStore } from '@/stores/robots';
import RobotNickname from '@/components/detail/RobotNickname.vue';
import RobotInfo from '@/components/detail/RobotInfo.vue';

const route = useRoute();
const router = useRouter();
const robotsStore = useRobotsStore();
const robotId = route.params.robotId;

const robot = computed(() => {
  return robotsStore.registered_robots.find(r => r.id == robotId) || null;
});

const showNicknameModal = ref(false);
const selectedRobotForNickname = ref(null);

const openNicknameModal = (robot) => {
  selectedRobotForNickname.value = { id: robot.id, nickname: robot.nickname || '' };
  showNicknameModal.value = true;
};

const closeNicknameModal = () => {
  showNicknameModal.value = false;
};

const setRobotNickname = (robotId, nickname) => {
  localStorage.setItem(`robot_nickname_${robotId}`, nickname);
  
  const robotIndex = robotsStore.registered_robots.findIndex(r => r.id === robotId);
  if (robotIndex !== -1) {
    robotsStore.registered_robots[robotIndex].nickname = nickname;
  }

  showNicknameModal.value = false;
};

const goBack = () => {
  router.push('/');
};

onMounted(() => {
  robotsStore.loadRobots();
});

watch(() => robotsStore.registered_robots, () => {
  if (!robot.value) {
    robotsStore.loadRobots();
  }
}, { deep: true });
</script>
