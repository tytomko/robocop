<template>
  <div class="robot-detail">
    <div class="robot-header">
      <h1 v-if="robot">{{ robot.nickname || robot.name }} ({{ robot.is_active ? '활성화' : '비활성화' }})</h1>
      <button @click="openNicknameModal(robot)" class="settings-btn">
        <i class="fas fa-cog"></i>
      </button>
    </div>

    <!-- RobotInfo 컴포넌트 사용 -->
    <RobotInfo v-if="robot" :robot="robot" />

    <button class="back-btn" @click="goBack">뒤로 가기</button>

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
import RobotInfo from '@/components/detail/RobotInfo.vue'; //

const route = useRoute();
const router = useRouter();
const robotsStore = useRobotsStore();
const robotId = route.params.robotId;

const robot = computed(() => {
  return robotsStore.registered_robots.find(r => r.id == robotId) || null;
});

// 닉네임 모달 상태
const showNicknameModal = ref(false);
const selectedRobotForNickname = ref(null);

// 로봇 닉네임 모달 열기
const openNicknameModal = (robot) => {
  selectedRobotForNickname.value = { id: robot.id, nickname: robot.nickname || '' };
  showNicknameModal.value = true;
};

// 닉네임 모달 닫기
const closeNicknameModal = () => {
  showNicknameModal.value = false;
};

// 닉네임 저장
const setRobotNickname = (robotId, nickname) => {
  localStorage.setItem(`robot_nickname_${robotId}`, nickname);

  // robots 상태 업데이트
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

<style scoped>
.robot-detail {
  padding: 20px;
  max-width: 600px;
  margin: 0 auto;
  overflow-y: auto; /* 세로 스크롤 활성화 */
  max-height: 80vh; /* 화면 높이를 넘어가지 않게 제한 */
}

.robot-header {
  display: flex;
  align-items: center; /* 세로 중앙 정렬 */
  gap: 10px; /* h1과 버튼 사이 여백 */
}

.back-btn {
  display: block;
  margin: 20px auto;
  padding: 10px 15px;
  background: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.back-btn:hover {
  background: #0056b3;
}

.settings-btn {
  background: none;
  border: none;
  cursor: pointer;
  font-size: 1.2rem;
}

.settings-btn i {
  color: #333; /* 아이콘 색상 */
}
</style>
