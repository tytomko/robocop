import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

export const useNotificationsStore = defineStore('notifications', () => {
  const alerts = ref(false);

  // alerts 상태를 업데이트하는 action
  function setAlertStatus(status) {
    alerts.value = status;
  }

  // alerts 상태를 토글하는 action
  function toggleAlert() {
    alerts.value = !alerts.value;
  }

  const notifications = ref([])
  const isNotificationsOpen = ref(false)
  const lastReadTimestamp = ref(localStorage.getItem('lastReadTimestamp') || null)

  // 읽지 않은 알림 개수 계산
  const unreadCount = computed(() => {
    if (!lastReadTimestamp.value) return notifications.value.length
    return notifications.value.filter(notification => !notification.isRead).length
  })

  // 알림 추가
  const addNotification = (message) => {
    const notification = {
      id: Date.now(),
      message,
      isRead: false,
      timestamp: new Date().toISOString()
    }
    notifications.value.unshift(notification)
    
    // 최대 5개까지만 유지
    if (notifications.value.length > 5) {
      notifications.value.pop()
    }

    // localStorage에 알림 저장
    saveNotificationsToStorage()
  }

  // 모든 알림을 읽음 상태로 변경
  const markAllAsRead = () => {
    notifications.value.forEach(notification => {
      notification.isRead = true
    })
    lastReadTimestamp.value = new Date().toISOString()
    
    // localStorage에 lastReadTimestamp 저장
    localStorage.setItem('lastReadTimestamp', lastReadTimestamp.value)
    saveNotificationsToStorage()
  }

  // 알림 창 토글
  const toggleNotifications = () => {
    isNotificationsOpen.value = !isNotificationsOpen.value
    if (isNotificationsOpen.value) {
      markAllAsRead()
    }
  }

  // localStorage에 알림 저장
  const saveNotificationsToStorage = () => {
    localStorage.setItem('notifications', JSON.stringify(notifications.value))
  }

  // localStorage에서 알림 로드
  const loadNotificationsFromStorage = () => {
    const savedNotifications = localStorage.getItem('notifications')
    if (savedNotifications) {
      notifications.value = JSON.parse(savedNotifications)
    }
  }

  // 컴포넌트 마운트 시 저장된 알림 로드
  loadNotificationsFromStorage()

  return {
    notifications,
    isNotificationsOpen,
    unreadCount,
    addNotification,
    markAllAsRead,
    toggleNotifications,
    alerts,
    setAlertStatus,
    toggleAlert
  }
})