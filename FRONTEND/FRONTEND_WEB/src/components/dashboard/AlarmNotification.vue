<template>
  <div :class="notificationWrapperClass">
    <!-- 알림 아이콘 -->
    <div class="relative cursor-pointer group" @click="notificationsStore.toggleNotifications">
      <div class="bg-transparent p-1 rounded-full transition-all duration-200 group-hover:bg-gray-700">
        <i :class="bellIconClass"></i>
        <span
          v-if="notificationsStore.unreadCount > 0"
          class="absolute -top-1 -right-1 bg-red-500 text-white text-xs font-medium rounded-full w-4 h-4 flex items-center justify-center"
        >
          {{ notificationsStore.unreadCount }}
        </span>
      </div>
    </div>

    <!-- 알림 드롭다운 -->
    <transition name="slide-fade">
      <div
        v-if="showDropdown"
        :class="dropdownClasses"
      >
        <div class="py-2 px-4 bg-gray-50 border-b border-gray-200">
          <h3 class="text-sm font-semibold text-gray-700">알림</h3>
        </div>
        <ul class="list-none p-0">
          <li
            v-for="notification in notificationsStore.notifications"
            :key="notification.id"
            class="flex items-center gap-3 p-3 border-b border-gray-100 cursor-pointer hover:bg-gray-50 transition-colors duration-200"
          >
            <div class="w-8 h-8 rounded-full overflow-hidden">
              <img
                :src="getNotificationImage(notification?.message)"
                alt="알림 아이콘"
                class="w-full h-full object-cover"
              />
            </div>
            <div class="flex-1">
              <p class="text-sm text-gray-800 font-medium leading-snug">{{ notification?.message.message || '알 수 없는 알림' }}</p>
              <span class="text-xs text-gray-500 mt-1 block">{{ getTimeAgo(notification.timestamp) }}</span>
            </div>
          </li>
        </ul>
        <div v-if="notificationsStore.notifications.length === 0" class="p-4 text-center text-sm text-gray-500">
          새로운 알림이 없습니다
        </div>
      </div>
    </transition>
  </div>
</template>

<script setup>
import { computed, onMounted, watch } from 'vue';
import { useNotificationsStore } from '@/stores/notifications';

const props = defineProps({
  isCollapsed: {  
    type: Boolean,
    default: false,
  },
  inline: {
    type: Boolean,
    default: false,
  }
});

const notificationsStore = useNotificationsStore();

const bellIconClass = computed(() => {
  return notificationsStore.isNotificationsOpen
    ? "fa-solid fa-bell text-white text-xl"
    : "fa-regular fa-bell text-white text-xl";
});

const getNotificationImage = (message) => {
  console.log('Notification message:', message, typeof message);  // 디버깅용
  if (!message || typeof message !== 'string') {
    return "/images/unknown.png";
  }

  if (message.includes("거수자를 발견하였습니다")) {
    return "/images/unknown.png";
  } else if (message.includes("신원이 확인되었습니다")) {
    return "/images/success.png";
  } else if (message.includes("새 로봇이 등록되었습니다")) {
    return "/images/robot.png";
  }
  return "/images/unknown.png";
};

// 인자로 timestamp를 받아서 현재 시간과 비교하여 상대적인 시간을 반환
const getTimeAgo = (timestamp) => {
  const now = new Date();
  const notificationDate = new Date(timestamp);
  const diff = now - notificationDate;
  
  const minutes = Math.floor(diff / 60000);
  const hours = Math.floor(diff / 3600000);
  const days = Math.floor(diff / 86400000);
  
  if (minutes < 60) {
    return "방금 전";
  } else if (hours < 24) {
    return `${hours}시간 전`;
  } else if (days < 7) {
    return `${days}일 전`;
  } else {
    return "1주 전";
  }
};

const notificationWrapperClass = computed(() => {
  if (props.inline) {
    return "relative"; 
  }
  return "fixed top-3 right-10 z-50";
});

const dropdownClasses = computed(() => {
  return props.isCollapsed
    ? "absolute top-full right-0 mt-2 bg-white rounded-lg border border-gray-200 w-80 max-h-[32rem] overflow-y-auto shadow-lg z-50 divide-y divide-gray-100"
    : "absolute top-full left-[-290px] mt-2 bg-white rounded-lg border border-gray-200 w-80 max-h-[32rem] overflow-y-auto shadow-lg z-50 divide-y divide-gray-100";
});

// notifications store 디버깅 추가
watch(() => notificationsStore.notifications, (newNotifications) => {
  console.log('Notifications updated:', newNotifications);
}, { deep: true });

// 드롭다운 표시 여부를 computed로 분리
const showDropdown = computed(() => {
  return notificationsStore.isNotificationsOpen;
});
</script>

<style scoped>
.slide-fade-enter-active,
.slide-fade-leave-active {
  transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1);
}

.slide-fade-enter-from,
.slide-fade-leave-to {
  transform: translateY(-10px);
  opacity: 0;
}

.slide-fade-enter-to,
.slide-fade-leave-from {
  transform: translateY(0);
  opacity: 1;
}
</style>