<template>
  <div class="robot-management">
    <div class="page-header">
      <h2>로봇 관리</h2>
      <button class="add-robot-btn" @click="showAddRobotModal">
        <i class="fas fa-plus"></i> 로봇 등록
      </button>
    </div>

    <!-- 로봇 목록 -->
    <div v-if="loading" class="loading">
      로봇 목록을 불러오는 중입니다...
    </div>
    <div v-else-if="robots.length === 0" class="empty-state">
      등록된 로봇이 없습니다. <br />
      상단의 "로봇 등록" 버튼을 클릭해 새 로봇을 등록하세요.
    </div>
    <div class="robot-grid">
      <div v-for="robot in robots" :key="robot.id" class="robot-card">
        <div class="robot-header">
          <img :src="robot.image || '/robot-placeholder.png'" alt="로봇 이미지" class="robot-image">
          <h3>{{ robot.name }}</h3>
          <div class="robot-status" :class="robot.status">{{ getStatusText(robot.status) }}</div>
        </div>
        <div class="robot-info">
          <div class="info-row">
            <span class="label">IP 주소:</span>
            <span class="value">{{ robot.ipAddress }}</span>
          </div>
          <div class="info-row">
            <span class="label">위치:</span>
            <span class="value">{{ robot.location }}</span>
          </div>
          <div class="info-row">
            <span class="label">배터리:</span>
            <div class="battery-indicator">
              <div class="battery-level" :style="{ width: robot.battery + '%', backgroundColor: getBatteryColor(robot.battery) }"></div>
              <span class="battery-text">{{ robot.battery }}%</span>
            </div>
          </div>
        </div>
        <div class="robot-actions">
          <button class="action-btn return" @click="returnRobot(robot.id)" :class="{ active: robot.status === 'charging' }">
            복귀 명령
          </button>
          <button class="action-btn emergency" @click="emergencyStop(robot.id)" :class="{ active: robot.status === 'emergency' }">
            비상 정지
          </button>
        </div>
      </div>
    </div>

    <!-- 로봇 등록 모달 -->
    <div class="modal" v-if="showModal" @click.self="closeModal">
      <div class="modal-content">
        <h3>로봇 등록</h3>
        <form @submit.prevent="handleAddRobot">
          <div class="form-group">
            <label>로봇명</label>
            <input v-model="newRobot.name" type="text" required>
          </div>
          <div class="form-group">
            <label>IP 주소</label>
            <input v-model="newRobot.ipAddress" type="text" required pattern="^(?:[0-9]{1,3}\.){3}[0-9]{1,3}$">
          </div>
          <div class="form-group">
            <label>로봇 이미지</label>
            <input type="file" @change="handleImageUpload" accept="image/*">
          </div>
          <div class="modal-actions">
            <button type="button" class="cancel-btn" @click="closeModal">취소</button>
            <button type="submit" class="submit-btn">등록</button>
          </div>
        </form>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'

// 상태 관리
const showModal = ref(false)
const robots = ref([])
const loading = ref(true)
const error = ref(null) 
const newRobot = ref({
  name: '',
  ipAddress: '',
  image: null
})

// 로봇 데이터 로드
const loadRobots = async () => {
  console.log('로봇 데이터를 로드합니다...');
  loading.value = true; // 로딩 상태 시작
  error.value = null
  try {
    // API 호출 로직
    await new Promise(resolve => setTimeout(resolve, 1000))
    robots.value = [
      {
        id: 1,
        name: 'Robot 1',
        status: 'active',
        ipAddress: '192.168.1.101',
        location: '1층 로비',
        battery: 85,
        image: null
      }
    ]
    console.log('로봇 데이터 로드 성공', robots.value);
  } catch (err) {
    error.value = '로봇 데이터를 불러오는데 실패했습니다.'
    console.error('로봇 데이터 로드 에러:', err)
  } finally {
    loading.value = false // 로딩 상태 종료
    console.log('로딩 상태 종료')
  }
}

// 상태 텍스트 변환
const getStatusText = (status) => {
  if (!status) return '알 수 없음'
  const statusMap = {
    active: '활동중',
    charging: '충전중',
    idle: '대기중',
    emergency: '비상정지'
  }
  return statusMap[status] || status
}

// 배터리 색상 계산
const getBatteryColor = (level) => {
  if (!level && level !== 0) return '#ccc'
  if (level > 60) return '#4CAF50'
  if (level > 20) return '#FFC107'
  return '#F44336'
}

// 모달 제어
const showAddRobotModal = () => {
  showModal.value = true
}

const closeModal = () => {
  showModal.value = false
  newRobot.value = {
    name: '',
    ipAddress: '',
    image: null
  }
}

// 이미지 업로드 처리
const handleImageUpload = (event) => {
  const file = event.target.files[0]
  if (file) {
    const reader = new FileReader()
    reader.onload = (e) => {
      newRobot.value.image = e.target.result
    }
    reader.readAsDataURL(file)
  }
}

// 로봇 등록 처리
const handleAddRobot = () => {
  // API 연동 후 구현
  robots.value.push({
    id: robots.value.length + 1,
    ...newRobot.value,
    status: 'idle',
    location: '대기장소',
    battery: 100
  })
  closeModal()
}

// 로봇 제어 함수
const returnRobot = async (robotId) => {
  if (!robotId) return
  try {
    // API 연동 후 구현
    const robot = robots.value.find(r => r.id === robotId)
    if (robot) {
      robot.status = robot.status === 'charging' ? 'idle' : 'charging'
    }
    console.log('복귀 명령:', robotId)
  } catch (err) {
    console.error('로봇 복귀 명령 에러:', err)
  }
}

const emergencyStop = async (robotId) => {
  if (!robotId) return
  try {
    const robot = robots.value.find(r => r.id === robotId)
    if (robot) {
      robot.status = robot.status === 'emergency' ? 'idle' : 'emergency'
    }
  } catch (err) {
    console.error('비상 정지 명령 에러:', err)
  }
}

onMounted(() => {
  loadRobots();
});
</script>

<style scoped>
.robot-management {
  padding: 20px;
}

.loading {
  text-align: center;
  padding: 20px;
  font-size: 1.2em;
  color: #666;
}

.empty-state {
  text-align: center;
  padding: 20px;
  font-size: 1.2em;
  color: #999;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.add-robot-btn {
  background-color: #4CAF50;
  color: white;
  border: none;
  padding: 10px 20px;
  border-radius: 4px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  text-align: center;
  gap: 8px;
}

.add-robot-btn i {
  display : inline-block;
  line-height : 1;
}

.add-robot-btn:hover {
  background-color: #45a049;
}

.robot-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 20px;
}

.robot-card {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  overflow: hidden;
}

.robot-header {
  padding: 15px;
  background: #f8f9fa;
  display: flex;
  align-items: center;
  gap: 10px;
}

.robot-image {
  width: 50px;
  height: 50px;
  border-radius: 4px;
  object-fit: cover;
}

.robot-status {
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 0.85em;
  margin-left: auto;
}

.robot-status.active { background: #e8f5e9; color: #2e7d32; }
.robot-status.charging { background: #fff3e0; color: #f57c00; }
.robot-status.idle { background: #f5f5f5; color: #616161; }
.robot-status.emergency { background: #ffebee; color: #c62828; }

.robot-info {
  padding: 15px;
}

.info-row {
  display: flex;
  justify-content: space-between;
  margin-bottom: 10px;
}

.info-row .label {
  color: #666;
}

.battery-indicator {
  width: 100px;
  height: 20px;
  background: #eee;
  border-radius: 10px;
  overflow: hidden;
  position: relative;
}

.battery-level {
  height: 100%;
  transition: width 0.3s ease;
}

.battery-text {
  position: absolute;
  width: 100%;
  text-align: center;
  color: #333;
  font-size: 0.85em;
  line-height: 20px;
}

.robot-actions {
  display: flex;
  gap: 10px;
  padding: 15px;
  background: #f8f9fa;
}

.action-btn {
  flex: 1;
  padding: 8px;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 0.9em;
}

.action-btn.return {
  background: #2196F3;
  color: white;
}

.action-btn.return:hover {
  background: #1976D2;
}

.action-btn.return:disabled {
  background: #90CAF9;
  cursor: not-allowed;
}

.action-btn.emergency {
  background: #F44336;
  color: white;
}

.action-btn.emergency:hover {
  background: #D32F2F;
}

.action-btn.emergency.active {
  background: #C62828;
}

/* 모달 스타일 */
.modal {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.modal-content {
  background: white;
  padding: 20px;
  border-radius: 8px;
  width: 100%;
  max-width: 400px;
}

.form-group {
  margin-bottom: 15px;
}

.form-group label {
  display: block;
  margin-bottom: 5px;
  color: #333;
}

.form-group input {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
}

.modal-actions {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
  margin-top: 20px;
}

.cancel-btn {
  padding: 8px 16px;
  border: 1px solid #ddd;
  background: red;
  border-radius: 4px;
  cursor: pointer;
}

.submit-btn {
  padding: 8px 16px;
  background: #4CAF50;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.submit-btn:hover {
  background: #45a049;
}
</style> 