self.addEventListener('push', (event) => {
  if (!event.data) return

  const notification = event.data.json()
  
  event.waitUntil(
    self.registration.showNotification(notification.title, {
      body: notification.body,
      icon: '/icon.png',
      badge: '/badge.png',
      data: notification.data
    })
  )
})

self.addEventListener('notificationclick', (event) => {
  event.notification.close()
  
  // 알림 클릭시 특정 페이지로 이동
  event.waitUntil(
    clients.openWindow('/')
  )
}) 