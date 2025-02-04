<template>
  <!-- 오른쪽 사이드바 -->
  <div class="right-sidebar">
    <div class="sidebar-sections">
      <div class="sidebar-section">
        <div class="section-header">
            <h3>지도</h3>
        </div>  

        <!-- 알림 아이콘 -->
        <div class="notification-icon" @click="toggleNotifications">
          <i :class="isNotificationsOpen ? 'fa-solid fa-bell bell-icon-active' : 'fa-regular fa-bell bell-icon'"></i>
          <span v-if="unreadCount > 0" class="notification-badge">
          {{ unreadCount }}
          </span>
        </div>

        <!-- 알림 토글창 -->
        <div v-if="isNotificationsOpen" class="notification-dropdown">
            <ul>
            <li v-for="(notification, index) in notifications" :key="index" :class="{ 'read': notification.isRead }">
                {{ notification.message }}
            </li>
            </ul>
        </div>

        <!-- 지도 섹션 -->
        <div class="map-container">
          <div class="robot-selector">
            <!-- 로봇 선택 -->
            <select v-model="robotsStore.selectedRobot" class="robot-select" @change="robotsStore.handleRobotSelection">
                <option value="">로봇 선택</option>
                <option v-for="robot in robotsStore.registered_robots" :key="robot.id" :value="robot.id">
                {{ robot.name }}
                </option>
            </select>
          </div>

          <!-- RobotMap.vue 컴포넌트 사용 -->
          <RobotMap />
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import RobotMap from '@/components/dashboard/RobotMap.vue';
import { ref, computed, onMounted, watch } from 'vue';
import { useRobotsStore } from '@/stores/robots';

// robotsStore 활용하기
const robotsStore = useRobotsStore();

// 알림 관련 변수
const notifications = ref([]) // 알림 목록
const isNotificationsOpen = ref(false) // 알림 창 열림 여부

// 알림을 추가하는 메소드 (최신 7개만 남도록)
const addNotification = (message) => {
  notifications.value.unshift({ message, isRead: false })  // 새 알림을 맨 앞에 추가
  if (notifications.value.length > 7) {  // 7개 이상이면, 가장 오래된 알림을 제거
    notifications.value.pop()
  }
}

// 모든 알림을 읽음 처리하는 메소드
const markAllAsRead = () => {
  notifications.value.forEach(notification => {
    notification.isRead = true
  })
}

// 읽지 않은 알림 개수
const unreadCount = computed(() => {
  return notifications.value.filter(notification => !notification.isRead).length
})


// 알림 토글 메소드
const toggleNotifications = () => {
  if (!isNotificationsOpen.value) {
    notifications.value.forEach((notification) => (notification.isRead = true))
  }
  isNotificationsOpen.value = !isNotificationsOpen.value
}

onMounted(() => {
  robotsStore.loadRobots() // 로봇 데이터 불러오기
  addNotification('거수자가 발견되었습니다.')
  const savedRobot = localStorage.getItem('selectedRobot')
  if (savedRobot) {
    robotsStore.selectedRobot = savedRobot
  }
})

</script>

<style scoped>
.right-sidebar {
  width: 400px;
  background: white;
  border-left: 1px solid #ddd;
  display: flex;
  flex-direction: column;
  flex-shrink: 0;
  overflow-y: auto;
}

.sidebar-sections {
  display: flex;
  flex-direction: column;
  gap: 1px;
  background: #f5f5f5;
  flex: 1;
}

.sidebar-section {
  background: white;
}

.section-header {
  display: flex;
  align-items: center;
  margin-bottom: 10px;
  padding: 15px 20px;
  border-bottom: solid #eee;
  background: white;
}

.section-header h3 {
  margin: 0;
  font-size: 16px;
  color: #333;
}

/* 로봇 선택 드롭다운 */
.robot-selector {
  padding: 8px 20px;
  margin-top: 8px;
  border-bottom: 1px solid #eee;
}

.robot-select {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  background: white;
  font-size: 14px;
}

/* 알림 아이콘 스타일링 */
.notification-icon {
  position: fixed;
  top: 13px;
  right: 20px;
  cursor: pointer;
  display: inline-block;
  z-index: 1;
}

/* 종 모양 아이콘 (Font Awesome 사용) */
.bell-icon,
.bell-icon-active {
  font-size: 24px;
  color: #333;
  transition: color 0.3s;
}

.bell-icon-active {
  color: #000;
}

/* 알림 배지 (알림 개수 표시) */
.notification-badge {
  position: absolute;
  top: -5px;
  right: -5px;
  background-color: red;
  color: white;
  border-radius: 50%;
  padding: 2px 6px;
  font-size: 12px;
  font-weight: bold;
}

/* 알림 드롭다운 (알림 목록) */
.notification-dropdown {
  position: fixed;
  top: 40px;
  right: 30px;
  background-color: #fff;
  border-radius: 8px;
  border: 1px solid #ccc;
  width: 300px;
  max-height: 250px;
  overflow-y: auto;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
  z-index: 10000;
  animation: slideIn 0.3s ease-in-out;
}

.notification-dropdown ul {
  list-style: none;
  margin: 0;
  padding: 0;
}

.notification-dropdown li {
  padding: 8px 10px;
  border-bottom: 1px solid #eee;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 10px;
  font-size: 14px;
}

.notification-dropdown li.read {
  background-color: #fff;
}

.notification-dropdown li:last-child {
  border-bottom: none;
}

.notification-content {
  display: flex;
  align-items: center;
  gap: 10px;
}

/* 애니메이션 */
@keyframes slideIn {
  from {
    transform: translateY(-20px);
    opacity: 0;
  }
  to {
    transform: translateY(0);
    opacity: 1;
  }
}

.map-container {
  padding: 0 20px;
  display: flex;
  flex-direction: column;
  gap: 20px;
}

</style>