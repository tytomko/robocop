<template>
  <div class="robot-settings">
    <div class="page-header">
      <h2>로봇 설정</h2>
      <div class="header-actions">
        <button class="action-btn" @click="saveSettings" :disabled="isSaving">
          {{ isSaving ? '저장 중...' : '설정 저장' }}
        </button>
      </div>
    </div>

    <div class="settings-grid">
      <!-- 기본 설정 -->
      <div class="settings-card">
        <h3>기본 설정</h3>
        <div class="settings-content">
          <div class="form-group">
            <label>로봇 이름</label>
            <input v-model="settings.name" type="text">
          </div>
          <div class="form-group">
            <label>IP 주소</label>
            <input v-model="settings.ip_address" type="text">
          </div>
          <div class="form-group">
            <label>설명</label>
            <textarea v-model="settings.description" rows="3"></textarea>
          </div>
        </div>
      </div>

      <!-- 동작 설정 -->
      <div class="settings-card">
        <h3>동작 설정</h3>
        <div class="settings-content">
          <div class="form-group">
            <label>최대 이동 속도 (m/s)</label>
            <input v-model="settings.max_speed" type="number" step="0.1" min="0">
          </div>
          <div class="form-group">
            <label>최대 회전 속도 (rad/s)</label>
            <input v-model="settings.max_angular_speed" type="number" step="0.1" min="0">
          </div>
          <div class="form-group">
            <label>가속도 제한 (m/s²)</label>
            <input v-model="settings.acceleration_limit" type="number" step="0.1" min="0">
          </div>
        </div>
      </div>

      <!-- 센서 설정 -->
      <div class="settings-card">
        <h3>센서 설정</h3>
        <div class="settings-content">
          <div class="form-group">
            <label>LiDAR 스캔 주기 (Hz)</label>
            <input v-model="settings.lidar_frequency" type="number" min="1">
          </div>
          <div class="form-group">
            <label>카메라 해상도</label>
            <select v-model="settings.camera_resolution">
              <option value="640x480">640 x 480</option>
              <option value="1280x720">1280 x 720</option>
              <option value="1920x1080">1920 x 1080</option>
            </select>
          </div>
          <div class="form-group">
            <label>카메라 FPS</label>
            <select v-model="settings.camera_fps">
              <option value="15">15 FPS</option>
              <option value="30">30 FPS</option>
              <option value="60">60 FPS</option>
            </select>
          </div>
        </div>
      </div>

      <!-- 안전 설정 -->
      <div class="settings-card">
        <h3>안전 설정</h3>
        <div class="settings-content">
          <div class="form-group">
            <label>충돌 방지 거리 (m)</label>
            <input v-model="settings.safety_distance" type="number" step="0.1" min="0">
          </div>
          <div class="form-group">
            <label>배터리 부족 경고 레벨 (%)</label>
            <input v-model="settings.low_battery_threshold" type="number" min="0" max="100">
          </div>
          <div class="form-group">
            <label>자동 충전 시작 레벨 (%)</label>
            <input v-model="settings.auto_charging_threshold" type="number" min="0" max="100">
          </div>
        </div>
      </div>

      <!-- 네트워크 설정 -->
      <div class="settings-card">
        <h3>네트워크 설정</h3>
        <div class="settings-content">
          <div class="form-group">
            <label>WebSocket 포트</label>
            <input v-model="settings.websocket_port" type="number" min="1024" max="65535">
          </div>
          <div class="form-group">
            <label>데이터 전송 주기 (ms)</label>
            <input v-model="settings.data_publish_rate" type="number" min="10">
          </div>
          <div class="form-group">
            <label>연결 재시도 간격 (ms)</label>
            <input v-model="settings.reconnect_interval" type="number" min="1000">
          </div>
        </div>
      </div>

      <!-- 고급 설정 -->
      <div class="settings-card">
        <h3>고급 설정</h3>
        <div class="settings-content">
          <div class="form-group">
            <label>디버그 모드</label>
            <div class="toggle-switch">
              <input 
                type="checkbox" 
                v-model="settings.debug_mode"
                :id="'debug-mode'"
              >
              <label :for="'debug-mode'"></label>
            </div>
          </div>
          <div class="form-group">
            <label>로그 레벨</label>
            <select v-model="settings.log_level">
              <option value="error">Error</option>
              <option value="warn">Warning</option>
              <option value="info">Info</option>
              <option value="debug">Debug</option>
            </select>
          </div>
          <div class="form-group">
            <label>자동 업데이트</label>
            <div class="toggle-switch">
              <input 
                type="checkbox" 
                v-model="settings.auto_update"
                :id="'auto-update'"
              >
              <label :for="'auto-update'"></label>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import { useRoute } from 'vue-router'
import { webSocketService } from '@/services/websocket'

const route = useRoute()
const isSaving = ref(false)

// 설정 상태 관리
const settings = ref({
  name: '',
  ip_address: '',
  description: '',
  max_speed: 1.0,
  max_angular_speed: 1.0,
  acceleration_limit: 0.5,
  lidar_frequency: 10,
  camera_resolution: '640x480',
  camera_fps: '30',
  safety_distance: 0.5,
  low_battery_threshold: 20,
  auto_charging_threshold: 30,
  websocket_port: 8000,
  data_publish_rate: 100,
  reconnect_interval: 3000,
  debug_mode: false,
  log_level: 'info',
  auto_update: true
})

// 설정 로드
const loadSettings = async () => {
  try {
    const response = await fetch(`/api/robots/${route.params.robotId}/settings`)
    if (!response.ok) throw new Error('설정을 불러올 수 없습니다.')
    const data = await response.json()
    settings.value = { ...settings.value, ...data }
  } catch (error) {
    console.error('설정 로드 실패:', error)
  }
}

// 설정 저장
const saveSettings = async () => {
  try {
    isSaving.value = true
    const response = await fetch(`/api/robots/${route.params.robotId}/settings`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(settings.value)
    })

    if (!response.ok) throw new Error('설정을 저장할 수 없습니다.')

    // 설정 변경 알림
    webSocketService.send('robot_settings_update', {
      robotId: route.params.robotId,
      settings: settings.value
    })

    alert('설정이 저장되었습니다.')
  } catch (error) {
    console.error('설정 저장 실패:', error)
    alert(error.message)
  } finally {
    isSaving.value = false
  }
}

onMounted(() => {
  loadSettings()
})
</script>

<style scoped>
.robot-settings {
  padding: 20px;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.header-actions {
  display: flex;
  gap: 10px;
}

.action-btn {
  padding: 8px 16px;
  border: none;
  border-radius: 4px;
  background: #2196F3;
  color: white;
  cursor: pointer;
}

.action-btn:hover:not(:disabled) {
  background: #1976D2;
}

.action-btn:disabled {
  background: #ccc;
  cursor: not-allowed;
}

.settings-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
}

.settings-card {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  overflow: hidden;
}

.settings-card h3 {
  margin: 0;
  padding: 15px;
  background: #f8f9fa;
  border-bottom: 1px solid #eee;
}

.settings-content {
  padding: 15px;
}

.form-group {
  margin-bottom: 15px;
}

.form-group:last-child {
  margin-bottom: 0;
}

.form-group label {
  display: block;
  margin-bottom: 5px;
  color: #333;
  font-weight: 500;
}

.form-group input[type="text"],
.form-group input[type="number"],
.form-group select,
.form-group textarea {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 1em;
}

.form-group input:focus,
.form-group select:focus,
.form-group textarea:focus {
  border-color: #2196F3;
  outline: none;
}

.toggle-switch {
  position: relative;
  width: 50px;
  height: 24px;
}

.toggle-switch input {
  opacity: 0;
  width: 0;
  height: 0;
}

.toggle-switch label {
  position: absolute;
  cursor: pointer;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: #ccc;
  transition: .4s;
  border-radius: 24px;
}

.toggle-switch label:before {
  position: absolute;
  content: "";
  height: 16px;
  width: 16px;
  left: 4px;
  bottom: 4px;
  background-color: white;
  transition: .4s;
  border-radius: 50%;
}

.toggle-switch input:checked + label {
  background-color: #2196F3;
}

.toggle-switch input:checked + label:before {
  transform: translateX(26px);
}
</style> 