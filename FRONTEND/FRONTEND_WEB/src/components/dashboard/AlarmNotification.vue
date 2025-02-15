<template>
  <div :class="notificationWrapperClass">
    <!-- 알림 아이콘 -->
    <div class="relative cursor-pointer group" @click="toggleNotifications">
      <div class="bg-transparent p-1 rounded-full transition-all duration-200 group-hover:bg-gray-700">
        <i :class="bellIconClass"></i>
        <span
          v-if="unreadCount > 0"
          class="absolute -top-1 -right-1 bg-red-500 text-white text-xs font-medium rounded-full w-4 h-4 flex items-center justify-center"
        >
          {{ unreadCount }}
        </span>
      </div>
    </div>

    <!-- 알림 드롭다운 -->
    <transition name="slide-fade">
      <div
        v-if="isNotificationsOpen"
        :class="dropdownClasses"
      >
        <div class="py-2 px-4 bg-gray-50 border-b border-gray-200">
          <h3 class="text-sm font-semibold text-gray-700">알림</h3>
        </div>
        <ul class="list-none p-0">
          <li
            v-for="(notification, index) in notifications"
            :key="index"
            class="flex items-center gap-3 p-3 border-b border-gray-100 cursor-pointer hover:bg-gray-50 transition-colors duration-200"
          >
            <div class="w-8 h-8 rounded-full overflow-hidden">
              <img
                :src="getNotificationImage(notification.message)"
                alt="알림 아이콘"
                class="w-full h-full object-cover"
              />
            </div>
            <div class="flex-1">
              <p class="text-sm text-gray-800 font-medium leading-snug">{{ notification.message }}</p>
              <span class="text-xs text-gray-500 mt-1 block">{{ getTimeAgo(index) }}</span>
            </div>
          </li>
        </ul>
        <div v-if="notifications.length === 0" class="p-4 text-center text-sm text-gray-500">
          새로운 알림이 없습니다
        </div>
      </div>
    </transition>
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
    ? "fa-solid fa-bell text-white text-xl"
    : "fa-regular fa-bell text-white text-xl";
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

onMounted(() => {
  addNotification("거수자를 발견하였습니다");
  addNotification("새 로봇이 등록되었습니다");
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