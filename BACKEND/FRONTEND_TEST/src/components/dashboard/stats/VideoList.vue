<template>
  <div class="video-list-container">
    <div class="page-header">
      <h2>영상 조회</h2>
      <div class="header-actions">
        <select v-model="selectedRobot" class="robot-select">
          <option value="">전체 로봇</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">
            {{ robot.name }}
          </option>
        </select>
        <div class="date-range">
          <input type="date" v-model="startDate" class="date-input" />
          <span>~</span>
          <input type="date" v-model="endDate" class="date-input" />
        </div>
        <button @click="searchVideos" class="search-button">
          <i class="fas fa-search"></i> 검색
        </button>
      </div>
    </div>

    <div class="video-grid">
      <div v-for="video in videos" :key="video.id" class="video-card">
        <div class="video-thumbnail" @click="playVideo(video)">
          <img :src="video.thumbnail" :alt="video.title" />
          <div class="video-duration">{{ formatDuration(video.duration) }}</div>
          <div class="video-play-button">
            <i class="fas fa-play"></i>
          </div>
        </div>
        <div class="video-info">
          <div class="video-title">{{ video.title }}</div>
          <div class="video-meta">
            <span>{{ video.robotName }}</span>
            <span>{{ formatDate(video.date) }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- 비디오 플레이어 모달 -->
    <div v-if="selectedVideo" class="video-modal" @click="closeVideo">
      <div class="video-modal-content" @click.stop>
        <div class="video-modal-header">
          <h3>{{ selectedVideo.title }}</h3>
          <button @click="closeVideo" class="close-button">
            <i class="fas fa-times"></i>
          </button>
        </div>
        <div class="video-player">
          <video ref="videoPlayer" controls>
            <source :src="selectedVideo.url" type="video/mp4" />
            브라우저가 비디오 재생을 지원하지 않습니다.
          </video>
        </div>
        <div class="video-modal-info">
          <div class="info-row">
            <span class="info-label">로봇:</span>
            <span>{{ selectedVideo.robotName }}</span>
          </div>
          <div class="info-row">
            <span class="info-label">날짜:</span>
            <span>{{ formatDate(selectedVideo.date) }}</span>
          </div>
          <div class="info-row">
            <span class="info-label">위치:</span>
            <span>{{ selectedVideo.location }}</span>
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
import { ref, onMounted } from 'vue'

// 상태 변수
const selectedRobot = ref('')
const startDate = ref('')
const endDate = ref('')
const loading = ref(false)
const videos = ref([])
const selectedVideo = ref(null)
const videoPlayer = ref(null)

// 임시 로봇 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' },
  { id: 'robot2', name: 'Robot 2' }
])

// 임시 비디오 데이터
const dummyVideos = [
  {
    id: 1,
    title: '로봇 순찰 영상 #1',
    thumbnail: 'https://via.placeholder.com/320x180',
    duration: 180,
    robotName: 'Robot 1',
    date: '2024-01-19T10:30:00',
    location: '1층 로비',
    url: 'https://example.com/video1.mp4'
  },
  {
    id: 2,
    title: '로봇 순찰 영상 #2',
    thumbnail: 'https://via.placeholder.com/320x180',
    duration: 240,
    robotName: 'Robot 2',
    date: '2024-01-19T11:15:00',
    location: '2층 사무실',
    url: 'https://example.com/video2.mp4'
  }
]

// 비디오 검색
const searchVideos = async () => {
  loading.value = true
  try {
    // 실제 API 호출로 대체 필요
    await new Promise(resolve => setTimeout(resolve, 1000))
    videos.value = dummyVideos
  } catch (error) {
    console.error('비디오 검색 중 오류:', error)
  } finally {
    loading.value = false
  }
}

// 비디오 재생
const playVideo = (video) => {
  selectedVideo.value = video
}

// 비디오 모달 닫기
const closeVideo = () => {
  if (videoPlayer.value) {
    videoPlayer.value.pause()
  }
  selectedVideo.value = null
}

// 날짜 포맷
const formatDate = (dateString) => {
  const date = new Date(dateString)
  return new Intl.DateTimeFormat('ko-KR', {
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit'
  }).format(date)
}

// 재생 시간 포맷
const formatDuration = (seconds) => {
  const minutes = Math.floor(seconds / 60)
  const remainingSeconds = seconds % 60
  return `${minutes}:${remainingSeconds.toString().padStart(2, '0')}`
}

onMounted(() => {
  // 오늘 날짜를 기본값으로 설정
  const today = new Date()
  const yesterday = new Date(today)
  yesterday.setDate(today.getDate() - 1)
  
  endDate.value = today.toISOString().split('T')[0]
  startDate.value = yesterday.toISOString().split('T')[0]
  
  searchVideos()
})
</script>

<style scoped>
.video-list-container {
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
  align-items: center;
}

.robot-select {
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  min-width: 150px;
}

.date-range {
  display: flex;
  align-items: center;
  gap: 8px;
}

.date-input {
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
}

.search-button {
  padding: 8px 16px;
  background: #007bff;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 8px;
}

.search-button:hover {
  background: #0056b3;
}

.video-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(320px, 1fr));
  gap: 20px;
  overflow-y: auto;
}

.video-card {
  background: white;
  border-radius: 8px;
  overflow: hidden;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.video-thumbnail {
  position: relative;
  aspect-ratio: 16/9;
  cursor: pointer;
}

.video-thumbnail img {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.video-duration {
  position: absolute;
  bottom: 8px;
  right: 8px;
  background: rgba(0, 0, 0, 0.8);
  color: white;
  padding: 2px 6px;
  border-radius: 4px;
  font-size: 12px;
}

.video-play-button {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  width: 48px;
  height: 48px;
  background: rgba(0, 0, 0, 0.7);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  font-size: 20px;
  opacity: 0;
  transition: opacity 0.3s;
}

.video-thumbnail:hover .video-play-button {
  opacity: 1;
}

.video-info {
  padding: 12px;
}

.video-title {
  font-weight: bold;
  margin-bottom: 4px;
}

.video-meta {
  display: flex;
  justify-content: space-between;
  color: #666;
  font-size: 12px;
}

.video-modal {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.8);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 1000;
}

.video-modal-content {
  background: white;
  border-radius: 8px;
  width: 90%;
  max-width: 1200px;
  max-height: 90vh;
  overflow: hidden;
  display: flex;
  flex-direction: column;
}

.video-modal-header {
  padding: 16px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid #ddd;
}

.video-modal-header h3 {
  margin: 0;
}

.close-button {
  background: none;
  border: none;
  font-size: 20px;
  cursor: pointer;
  color: #666;
}

.video-player {
  flex: 1;
  background: black;
}

.video-player video {
  width: 100%;
  height: 100%;
}

.video-modal-info {
  padding: 16px;
  background: #f8f9fa;
}

.info-row {
  display: flex;
  margin-bottom: 8px;
}

.info-row:last-child {
  margin-bottom: 0;
}

.info-label {
  width: 80px;
  color: #666;
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