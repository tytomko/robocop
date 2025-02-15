<template>
  <div class="robot-detail max-w-xl mx-auto p-5 overflow-y-auto max-h-[80vh]">
    <div class="robot-header flex items-center gap-3">
      <h1 v-if="robot" class="text-xl font-bold">
        {{ robot.nickname || robot.name }} 
        <span class="text-sm" :class="robot.isActive ? 'text-green-500' : 'text-red-500'">
          ({{ robot.isActive ? '활성화' : '비활성화' }})
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
import axios from 'axios'

const route = useRoute();
const router = useRouter();
const robotsStore = useRobotsStore();
const seq = route.params.seq;

const robot = computed(() => {
  return robotsStore.registered_robots.find(r => r.seq == seq) || null;
});

const showNicknameModal = ref(false);
const selectedRobotForNickname = ref(null);

const openNicknameModal = (robot) => {
  selectedRobotForNickname.value = { seq: robot.seq, nickname: robot.nickname || '' };
  showNicknameModal.value = true;
};

const closeNicknameModal = () => {
  showNicknameModal.value = false;
};

// 닉네임 저장 시 DB에 반영하는 함수 추가
const setRobotNickname = async (seq, nickname) => {
  try {
    // 백엔드 API 호출 (PUT 요청)
    await axios.patch(`https://robocop-backend-app.fly.dev/api/v1/robots/${seq}`, {
      nickname: nickname
    });

    // 성공적으로 업데이트하면 로컬 데이터도 반영
    const robotIndex = robotsStore.registered_robots.findIndex(r => r.seq === seq);
    if (robotIndex !== -1) {
      robotsStore.registered_robots[robotIndex].nickname = nickname;
    }

    // 로컬 스토리지에도 저장 (선택한 로봇이 있을 경우)
    if (robotsStore.selectedRobot === seq) {
      localStorage.setItem(`robot_nickname_${seq}`, nickname);
    }

    // 최신 로봇 데이터 다시 불러오기
    await robotsStore.loadRobots();

    // 모달 닫기
    showNicknameModal.value = false;
  } catch (error) {
    console.error('로봇 닉네임 업데이트 실패:', error);
    alert('로봇 닉네임을 업데이트하는 중 오류가 발생했습니다.');
  }
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
