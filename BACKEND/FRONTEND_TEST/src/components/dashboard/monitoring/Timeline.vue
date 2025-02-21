<template>
  <div class="timeline-container">
    <div class="page-header">
      <h2>타임라인</h2>
      <div class="header-actions">
        <select v-model="selectedRobot" class="robot-select">
          <option value="">전체 로봇</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
      </div>
    </div>

    <div class="timeline">
      <div v-for="event in events" :key="event.id" class="timeline-item" :class="event.type">
        <div class="timeline-marker"></div>
        <div class="timeline-content">
          <div class="event-header">
            <span class="event-time">{{ formatDateTime(event.timestamp) }}</span>
            <span class="event-robot">{{ event.robotName }}</span>
          </div>
          <div class="event-body">
            <p class="event-message">{{ event.message }}</p>
            <div v-if="event.image" class="event-image">
              <img :src="event.image" :alt="event.message">
            </div>
          </div>
          <div class="event-footer">
            <span class="event-location">{{ event.location }}</span>
          </div>
        </div>
      </div>
    </div>

    <div v-if="loading" class="loading-overlay">
      <div class="loading-spinner"></div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import { webSocketService } from '@/services/websocket'

const selectedRobot = ref('')
const events = ref([])
const loading = ref(false)

// 임시 로봇 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' }
])

// 날짜 포맷팅
const formatDateTime = (timestamp) => {
  const date = new Date(timestamp)
  return `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}-${String(date.getDate()).padStart(2, '0')} ${String(date.getHours()).padStart(2, '0')}:${String(date.getMinutes()).padStart(2, '0')}`
}

// 이벤트 데이터 가져오기
const fetchEvents = async () => {
  loading.value = true
  try {
    // API 호출 시뮬레이션
    const response = await new Promise(resolve => {
      setTimeout(() => {
        resolve({
          events: [
            {
              id: 1,
              type: 'warning',
              timestamp: new Date().getTime(),
              robotName: 'Robot 1',
              message: '배터리 잔량 20% 미만',
              location: '1층 로비',
              image: null
            },
            {
              id: 2,
              type: 'info',
              timestamp: new Date().getTime() - 1000 * 60 * 5,
              robotName: 'Robot 2',
              message: '순찰 임무 시작',
              location: '2층 사무실',
              image: null
            },
            {
              id: 3,
              type: 'error',
              timestamp: new Date().getTime() - 1000 * 60 * 15,
              robotName: 'Robot 1',
              message: '장애물 감지로 인한 경로 이탈',
              location: '1층 회의실',
              image: '/images/obstacle.jpg'
            }
          ]
        })
      }, 500)
    })
    events.value = response.events
  } catch (error) {
    console.error('이벤트 데이터 로드 실패:', error)
  } finally {
    loading.value = false
  }
}

// 웹소켓 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/api/v1/robots/ws')
    
    // 이벤트 구독
    webSocketService.subscribe('timeline/events', (data) => {
      if (!data) return
      
      // 새 이벤트를 목록 맨 앞에 추가
      events.value.unshift({
        id: Date.now(),
        ...data
      })
    }, '/api/v1/robots/ws')
  } catch (error) {
    console.error('웹소켓 연결 실패:', error)
  }
}

// 로봇 선택 변경 감지
watch(selectedRobot, () => {
  fetchEvents()
})

onMounted(async () => {
  await setupWebSocket()
  await fetchEvents()
})
</script>

<style scoped>
.timeline-container {
  padding: 20px;
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.header-actions {
  display: flex;
  gap: 10px;
}

.robot-select {
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  min-width: 150px;
}

.timeline {
  flex: 1;
  overflow-y: auto;
  padding: 20px;
  position: relative;
}

.timeline::before {
  content: '';
  position: absolute;
  left: 30px;
  top: 0;
  bottom: 0;
  width: 2px;
  background: #ddd;
}

.timeline-item {
  position: relative;
  margin-bottom: 30px;
  padding-left: 50px;
}

.timeline-marker {
  position: absolute;
  left: 22px;
  width: 16px;
  height: 16px;
  border-radius: 50%;
  background: #fff;
  border: 2px solid #ddd;
}

.timeline-item.info .timeline-marker {
  border-color: #007bff;
}

.timeline-item.warning .timeline-marker {
  border-color: #ffc107;
}

.timeline-item.error .timeline-marker {
  border-color: #dc3545;
}

.timeline-content {
  background: white;
  padding: 15px;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.event-header {
  display: flex;
  justify-content: space-between;
  margin-bottom: 10px;
}

.event-time {
  color: #666;
  font-size: 0.9em;
}

.event-robot {
  font-weight: 500;
  color: #333;
}

.event-body {
  margin-bottom: 10px;
}

.event-message {
  margin: 0;
  color: #333;
}

.event-image {
  margin-top: 10px;
}

.event-image img {
  max-width: 100%;
  border-radius: 4px;
}

.event-footer {
  color: #666;
  font-size: 0.9em;
}

.loading-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(255, 255, 255, 0.8);
  display: flex;
  justify-content: center;
  align-items: center;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 3px solid #f3f3f3;
  border-top: 3px solid #007bff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}
</style> 