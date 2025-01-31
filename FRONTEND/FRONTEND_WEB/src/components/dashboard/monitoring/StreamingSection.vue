<template>
    <!-- 오른쪽 실시간 모니터링 사이드바 -->
    <div class="right-sidebar">
        <div class="sidebar-sections">
            <div class="sidebar-section">
            <div class="section-header">
                <h3>실시간 스트리밍</h3>
            </div>

            <!-- 알림 아이콘 -->
        <div class="notification-icon" @click="toggleNotifications">
            <!-- 빈 종 또는 가득 찬 종으로 상태에 따라 아이콘 변경 -->
            <i :class="isNotificationsOpen ? 'fa-solid fa-bell bell-icon-active' : 'fa-regular fa-bell bell-icon'"></i>
            <!-- 읽지 않은 알림 개수 표시 -->
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

            <!-- 실시간 모니터링 섹션 -->
            <div class="video-container">
                <div class="robot-selector">
                <!-- 로봇 선택 -->
                <select v-model="robotsStore.selectedRobot" class="robot-select" @change="robotsStore.handleRobotSelection">
                    <option value="">로봇 선택</option>
                    <option v-for="robot in robotsStore.registered_robots" :key="robot.id" :value="robot.id">
                    {{ robot.name }}
                    </option>
                </select>
                </div>
                <!-- 선택된 로봇 카메라 -->
                <div v-if="robotsStore.selectedRobot" class="camera-sections">
                <CameraView cameraName="프론트캠" :cameraStatus="'연결 대기 중'" />
                <CameraView cameraName="리어캠" :cameraStatus="'연결 대기 중'" />
                </div>
                <div v-else class="no-robot-selected">로봇을 선택해주세요</div>
            </div>
            </div>
        </div>
    </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue'
import { useRouter, useRoute } from 'vue-router'
import { useRobotsStore } from '@/stores/robots'
import CameraView from '@/components/dashboard/monitoring/CameraView.vue'

// robotsStore 활용하기
const robotsStore = useRobotsStore();
const router = useRouter();
const route = useRoute();
const activeMenu = ref('');

// robotsStore에 등록된 로봇 불러오기
watch(() => robotsStore.selectedRobot, (newRobotId) => {
  if (newRobotId) {
    localStorage.setItem('selectedRobot', newRobotId);
  }
}, { immediate: true });

// 알림 관련 변수
const notifications = ref([]) // 알림 목록
const isNotificationsOpen = ref(false) // 알림 창 열림 여부

// 알림을 추가하는 메소드 (최신 5개만 남도록)
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

.alert-section {
  padding: 20px;
  background: white;
}

.alert-section h3 {
  margin: 0 0 15px 0;
  color: #333;
}

.alert-list {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.alert-item {
  padding: 12px;
  border-radius: 4px;
  display: flex;
  flex-direction: column;
  gap: 5px;
}

.alert-item.warning {
  background-color: #fff3cd;
  border: 1px solid #ffeeba;
  color: #856404;
}

.alert-item.info {
  background-color: #cce5ff;
  border: 1px solid #b8daff;
  color: #004085;
}

.alert-item .time {
  font-size: 0.85em;
  opacity: 0.8;
}

.alert-item .message {
  font-weight: 500;
}

.divider {
  height: 1px;
  background: #ddd;
  margin: 0;
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

.section-handle {
  cursor: move;
  padding: 0 10px;
  color: #666;
  font-size: 18px;
  margin-right: 10px;
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

.no-robot-selected {
  padding: 20px;
  text-align: center;
  color: #666;
}

/* 드롭다운 스타일 개선 */
.robot-select {
  appearance: none;
  background-image: url("data:image/svg+xml;charset=UTF-8,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3e%3cpolyline points='6 9 12 15 18 9'%3e%3c/polyline%3e%3c/svg%3e");
  background-repeat: no-repeat;
  background-position: right 8px center;
  background-size: 16px;
  padding-right: 32px;
}

.robot-select:focus {
  outline: none;
  border-color: #007bff;
}

/* 알림 아이콘 스타일링 */
.notification-icon {
  position: fixed; /* 오른쪽 끝으로 고정 */
  top: 14px;
  right: 20px; /* 화면의 가장 오른쪽 끝 */
  cursor: pointer;
  display: inline-block;
  z-index: 99; /* 다른 요소보다 위에 위치하도록 설정 */
}

/* 종 모양 아이콘 (Font Awesome 사용) */
.bell-icon,
.bell-icon-active {
  font-size: 24px;
  color: #333; /* 기본 색상 */
  transition: color 0.3s;
}

.bell-icon-active {
  color: #000; /* 클릭 시 까만색으로 변경 */
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
  position: fixed; /* 화면 고정 */
  top: 40px; /* 종 바로 아래 */
  right: 20px;
  background-color: white;
  border: 1px solid #ccc;
  width: 200px;
  max-height: 175px; /* 높이 제한 */
  overflow-y: auto; /* 내용이 넘치면 스크롤 */
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
  z-index: 10000; /* 다른 요소보다 위에 나타나도록 설정 */
}

.notification-dropdown ul {
  list-style: none;
  margin: 0;
  padding: 0;
}

.notification-dropdown li {
  padding: 6px 8px; /* 간격 줄임: 위아래 6px, 좌우 8px */
  border-bottom: 1px solid #eee;
  cursor: pointer;
  font-size: 14px;  /* 텍스트 크기 약간 줄임 */
}

/* 읽은 알림 스타일 */
.notification-dropdown li.read {
  background-color: #f0f0f0;
}

.notification-dropdown li:last-child {
  border-bottom: none;
}

.video-container {
  padding: 0 20px;
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.camera-section {
  background: #1a1a1a;
  border-radius: 8px;
  overflow: hidden;
  margin-bottom: 1rem;
}

.camera-section h3 {
  padding: 0.5rem 1rem;
  margin: 0;
  background: #2a2a2a;
  color: white;
}
</style>