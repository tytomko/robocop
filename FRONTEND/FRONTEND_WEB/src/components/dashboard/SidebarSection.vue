<template>
  <!-- 오른쪽 사이드바 -->
  <div class="w-96 bg-white border-l border-gray-300 flex flex-col overflow-y-auto flex-shrink-0">
    <div class="flex flex-col gap-1 bg-gray-100 flex-1">
      <div class="bg-white">
        <div class="flex items-center mb-2 p-4 border-b border-gray-200 bg-white">
          <h3 class="m-0 text-lg text-gray-800">지도</h3>
        </div>

        <!-- 알림 아이콘 -->
        <div class="fixed top-3 right-5 cursor-pointer z-10" @click="toggleNotifications">
          <i :class="isNotificationsOpen ? 'fa-solid fa-bell text-black' : 'fa-regular fa-bell text-gray-700 text-xl transition-colors'" ></i>
          <span v-if="unreadCount > 0" class="absolute top-0 right-0 bg-red-500 text-white text-xs font-bold rounded-full px-2">
            {{ unreadCount }}
          </span>
        </div>

        <!-- 알림 토글창 -->
        <div v-if="isNotificationsOpen" class="fixed top-10 right-8 bg-white rounded-lg border border-gray-300 w-72 max-h-64 overflow-y-auto shadow-lg z-50 animate-slideIn">
          <ul class="list-none m-0 p-0">
            <li v-for="(notification, index) in notifications" :key="index" class="p-2 border-b border-gray-200 cursor-pointer text-sm hover:bg-gray-100" :class="{ 'bg-gray-100': notification.isRead }">
              {{ notification.message }}
            </li>
          </ul>
        </div>

        <!-- 지도 섹션 -->
        <div class="p-4 flex flex-col gap-5">
          <RobotMap />
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import RobotMap from '@/components/map/RobotMap.vue';
import { ref, computed, onMounted } from 'vue';
import { useRobotsStore } from '@/stores/robots';

const robotsStore = useRobotsStore();
const notifications = ref([]);
const isNotificationsOpen = ref(false);

const addNotification = (message) => {
  notifications.value.unshift({ message, isRead: false });
  if (notifications.value.length > 7) {
    notifications.value.pop();
  }
};

const unreadCount = computed(() => {
  return notifications.value.filter(notification => !notification.isRead).length;
});

const toggleNotifications = () => {
  if (!isNotificationsOpen.value) {
    notifications.value.forEach(notification => (notification.isRead = true));
  }
  isNotificationsOpen.value = !isNotificationsOpen.value;
};

onMounted(() => {
  robotsStore.loadRobots();
  addNotification('거수자가 발견되었습니다.');
  const savedRobot = localStorage.getItem('selectedRobot');
  if (savedRobot) {
    robotsStore.selectedRobot = savedRobot;
  }
});
</script>

<style scoped>
@keyframes slideIn {
  from {
    transform: translateY(-10px);
    opacity: 0;
  }
  to {
    transform: translateY(0);
    opacity: 1;
  }
}
</style>