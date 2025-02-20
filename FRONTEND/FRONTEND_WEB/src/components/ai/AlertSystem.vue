<template>
  <div>
    <!-- 알림 팝업 -->
    <Transition name="slide-fade">
      <div v-if="robotsStore.alerts" 
           class="fixed top-[55px] left-0 right-0 bg-red-600 text-white py-2 px-4 shadow-lg z-50">
        <div class="flex items-center justify-center">
          <svg xmlns="http://www.w3.org/2000/svg" class="h-6 w-6 mr-2 animate-pulse" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
          </svg>
          <span class="font-bold text-lg">인증에 실패한 거수자 발견!</span>
        </div>
      </div>
    </Transition>

    <!-- 경보 해제 버튼 -->
    <Transition name="fade">
      <button v-if="robotsStore.alerts"
              @click="confirmDisableAlert"
              class="ml-4 bg-red-600 hover:bg-red-700 text-white px-4 py-1 rounded-md flex items-center">
        <svg xmlns="http://www.w3.org/2000/svg" class="h-5 w-5 mr-1" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M6 18L18 6M6 6l12 12" />
        </svg>
        경보 해제
      </button>
    </Transition>

    <!-- 확인 모달 -->
    <Transition name="fade">
      <div v-if="showConfirmModal" 
           class="fixed inset-0 flex items-center justify-center z-50">
        <div class="absolute inset-0 bg-black bg-opacity-50"></div>
        <div class="relative bg-white rounded-lg p-6 max-w-sm mx-4">
          <h3 class="text-xl font-bold mb-4 text-gray-900">경보 해제 확인</h3>
          <p class="mb-6 text-gray-700">경보를 해제하시겠습니까?</p>
          <div class="flex justify-end space-x-4">
            <button @click="cancelDisableAlert"
                    class="px-4 py-2 border border-gray-300 rounded-md hover:bg-gray-100 text-gray-700">
              취소
            </button>
            <button @click="disableAlert"
                    :disabled="isLoading"
                    class="px-4 py-2 bg-red-600 text-white rounded-md hover:bg-red-700 disabled:bg-red-400 disabled:cursor-not-allowed">
              {{ isLoading ? '처리 중...' : '확인' }}
            </button>
          </div>
        </div>
      </div>
    </Transition>
  </div>
</template>

<script setup>
import { ref, watch } from 'vue';
import { useRobotsStore } from '@/stores/robots';
import axios from 'axios';

const robotsStore = useRobotsStore();
const showConfirmModal = ref(false);
const isLoading = ref(false);

// alert 상태 변화 감지하여 navbar 스타일 변경
watch(() => robotsStore.alerts, (newValue) => {
  if (newValue) {
    document.querySelector('nav')?.classList.add('alert-active');
  } else {
    document.querySelector('nav')?.classList.remove('alert-active');
  }
});

// 경보 해제 확인 모달 표시
const confirmDisableAlert = () => {
  showConfirmModal.value = true;
};

// 경보 해제 취소
const cancelDisableAlert = () => {
  showConfirmModal.value = false;
};

// 경보 해제 실행
const disableAlert = async () => {
  try {
    isLoading.value = true;
    
    // API 호출
    await axios.post('https://robocopbackendssafy.duckdns.org/api/v1/alert-off');
    
    // 성공하면 로컬 상태 업데이트
    robotsStore.alerts = false;
    showConfirmModal.value = false;
  } catch (error) {
    console.error('경보 해제 실패:', error);
    alert('경보 해제에 실패했습니다. 다시 시도해주세요.');
  } finally {
    isLoading.value = false;
  }
};
</script>

<style scoped>
.slide-fade-enter-active,
.slide-fade-leave-active {
  transition: all 0.3s ease;
}

.slide-fade-enter-from,
.slide-fade-leave-to {
  transform: translateY(-100%);
  opacity: 0;
}

.fade-enter-active,
.fade-leave-active {
  transition: opacity 0.3s ease;
}

.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}

:global(nav.alert-active) {
  animation: alertBlink 1s infinite;
}

@keyframes alertBlink {
  0%, 100% {
    background-color: #111827;
  }
  50% {
    background-color: #dc2626;
  }
}
</style>