<template>
    <!-- 알림 아이콘 (오른쪽 최상단 고정) -->
    <div class="fixed top-4 right-4 cursor-pointer z-50 flex items-center justify-center" @click="toggleNotifications">
      <!-- 종 아이콘 (빈 종 ↔ 꽉 찬 종) -->
      <div class="relative">
        <i :class="bellIconClass"></i>
        <!-- 알림 배지 -->
        <span v-if="unreadCount > 0"
          class="absolute -top-2 -right-2 bg-red-600 text-white text-xs font-bold rounded-full w-5 h-5 flex items-center justify-center shadow-md">
          {{ unreadCount }}
        </span>
      </div>
    </div>
  
    <!-- 알림 토글창 -->
    <div v-if="isNotificationsOpen"
      class="fixed top-12 right-6 bg-white rounded-lg border border-gray-300 w-80 max-h-96 overflow-y-auto shadow-lg z-50 animate-slideIn">
      <ul class="list-none p-0">
        <li v-for="(notification, index) in notifications" :key="index"
          class="flex items-center gap-3 p-3 border-b border-gray-200 cursor-pointer text-sm hover:bg-gray-100">
          <!-- 알림 이미지 -->
          <img :src="getNotificationImage(notification.message)" alt="알림 아이콘"
            class="w-10 h-10 rounded-full object-cover">
          <!-- 알림 내용 -->
          <div class="flex-1">
            <p class="text-gray-800 font-medium">{{ notification.message }}</p>
            <span class="text-xs text-gray-500">{{ getTimeAgo(index) }}</span>
          </div>
        </li>
      </ul>
    </div>
  </template>
  
  <script setup>
  import { ref, computed, onMounted } from 'vue';
  
  const notifications = ref([]);
  const isNotificationsOpen = ref(false);
  
  /** 🔔 종 모양 동적 변경 */
  const bellIconClass = computed(() => {
    return isNotificationsOpen.value
      ? "fa-solid fa-bell text-black text-2xl"
      : "fa-regular fa-bell text-black text-2xl";
  });
  
  /** 알림 이미지 동적 변경 */
  const getNotificationImage = (message) => {
    if (message.includes("거수자를 발견하였습니다")) {
      return "/images/unknown.png";
    } else if (message.includes("새 로봇이 등록되었습니다")) {
      return "/images/robot.png";
    }
    return "/images/unknown.png";
  };
  
  /** 알림 시간 (더미 데이터) */
  const getTimeAgo = (index) => {
    const timeList = ["방금 전", "1시간 전", "3시간 전", "1일 전", "3일 전", "1주 전"];
    return timeList[index % timeList.length];
  };
  
  /** 알림 추가 */
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
    addNotification("거수자를 발견하였습니다");
    addNotification("새 로봇이 등록되었습니다");
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
