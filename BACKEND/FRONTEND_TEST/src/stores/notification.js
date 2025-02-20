import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useNotificationStore = defineStore('notification', () => {
  const notifications = ref([])
  const unreadCount = ref(0)

  // 서비스 워커 등록
  const registerServiceWorker = async () => {
    try {
      // 푸시 알림 권한 요청
      const permission = await Notification.requestPermission()
      if (permission !== 'granted') {
        throw new Error('알림 권한이 거부되었습니다.')
      }

      // 서비스 워커 등록
      const registration = await navigator.serviceWorker.register('/sw.js')
      
      // 서버에 구독 정보 전송
      const subscription = await registration.pushManager.subscribe({
        userVisibleOnly: true,
        applicationServerKey: '서버의_공개키' // 백엔드에서 제공하는 VAPID 공개키
      })

      // 구독 정보를 서버에 전송
      await fetch('/api/notifications/subscribe', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(subscription)
      })
    } catch (error) {
      console.error('Push 알림 등록 실패:', error)
    }
  }

  // 새 알림 추가
  const addNotification = (notification) => {
    notifications.value.unshift(notification)
    unreadCount.value++
  }

  // 알림 삭제
  const removeNotification = (id) => {
    notifications.value = notifications.value.filter(n => n.id !== id)
    if (unreadCount.value > 0) unreadCount.value--
  }

  // 모든 알림 삭제
  const clearAllNotifications = () => {
    notifications.value = []
    unreadCount.value = 0
  }

  return {
    notifications,
    unreadCount,
    registerServiceWorker,
    addNotification,
    removeNotification,
    clearAllNotifications
  }
}) 