<template>
  <div :class="notificationWrapperClass">
    <!-- 알림 아이콘 -->
    <div class="relative cursor-pointer" @click="toggleNotifications">
      <div class="bg-white p-1 rounded-full">
        <i :class="bellIconClass"></i>
        <span v-if="unreadCount > 0"
          class="absolute -top-2 -right-2 bg-red-600 text-white text-xs font-bold rounded-full w-5 h-5 flex items-center justify-center shadow-md">
          {{ unreadCount }}
        </span>
      </div>
    </div>

    <!-- 알림 드롭다운 -->
    <div v-if="isNotificationsOpen" :class="dropdownClasses">
      <ul class="list-none p-0">
        <li v-for="(notification, index) in notifications" :key="index"
          class="flex items-center gap-3 p-3 border-b border-gray-200 cursor-pointer text-sm hover:bg-gray-100">
          <img :src="getNotificationImage(notification.message)" alt="알림 아이콘"
            class="w-10 h-10 rounded-full object-cover">
          <div class="flex-1">
            <p class="text-gray-800 font-medium">{{ notification.message }}</p>
            <span class="text-xs text-gray-500">{{ getTimeAgo(index) }}</span>
          </div>
        </li>
      </ul>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted } from 'vue';

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

const notifications = ref([]);
const isNotificationsOpen = ref(false);

const bellIconClass = computed(() => {
  return isNotificationsOpen.value
    ? "fa-solid fa-bell text-black text-2xl"
    : "fa-regular fa-bell text-black text-2xl";
});

const getNotificationImage = (message) => {
  if (message.includes("거수자를 발견하였습니다")) {
    return "/images/unknown.png";
  } else if (message.includes("새 로봇이 등록되었습니다")) {
    return "/images/robot.png";
  }
  return "/images/unknown.png";
};

const getTimeAgo = (index) => {
  const timeList = ["방금 전", "1시간 전", "3시간 전", "1일 전", "3일 전", "1주 전"];
  return timeList[index % timeList.length];
};

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

// 알림 아이콘 위치 조정
const notificationWrapperClass = computed(() => {
  if (props.inline) {
    return "relative ml-4"; // 네비게이션 바 내부일 때
  }
  return "fixed top-2 right-10 z-50"; // 사이드바가 축소된 경우
});

// 알림 드롭다운 위치 조정
const dropdownClasses = computed(() => {
  return props.isCollapsed
    ? "absolute top-full right-0 mt-2 bg-white rounded-lg border border-gray-300 w-80 max-h-96 overflow-y-auto shadow-lg z-50 animate-slideIn"
    : "absolute top-full left-[-290px] mt-2 bg-white rounded-lg border border-gray-300 w-80 max-h-96 overflow-y-auto shadow-lg z-50 animate-slideIn";
});

onMounted(() => {
  addNotification("거수자를 발견하였습니다");
  addNotification("새 로봇이 등록되었습니다");
});
</script>

<style scoped>
@keyframes slideIn {
  from { transform: translateY(-10px); opacity: 0; }
  to { transform: translateY(0); opacity: 1; }
}
</style>
