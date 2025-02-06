<template>
  <div class="realtime-monitoring">
    <div class="monitoring-header">
      <h2>실시간 모니터링</h2>
    </div>

    <div class="monitoring-sections">
      <div class="monitoring-section">
        <div class="section-header">
          <h4>로봇 목록</h4>
        </div>
            
        <!-- 로봇 상태 목록 -->
        <div class="robot-status-list">
          <div v-for="robot in visibleRobots" 
              :key="robot.id" 
              class="robot-status-card">

              <!-- X 버튼 -->
              <button class="close-btnx" @click="hideRobot(robot.id)">✖</button>

            <div class="robot-header">
              <span class="robot-name"><strong>{{ robot.nickname || robot.name }}</strong></span>

              <span class="status-badge" :class="getStatusClass(robot.status)">
                {{ getStatusLabel(robot.status) }}
              </span>
            </div>

            <div class="status-details">
              <div class="status-item">
                <span class="label">배터리</span>
                <div class="battery-indicator">
                  <div class="battery-level"
                  :style="{
                    width: robot.battery + '%',
                    backgroundColor: getBatteryColor(robot.battery)}">
                  </div>
                  <span class="battery-text">{{ robot.battery }}%</span>
                </div>
              </div>

              <div class="status-item">
                <span class="label">현재 위치</span>
                <span class="value">{{ robot.location }}</span>
              </div>

              <div class="status-item">
                <span class="label">IP 주소</span>
                <span class="value">{{ robot.ipAddress }}</span>
              </div>

            </div>

              <div class="robot-actions">
                <button class="action-btn return" @click="returnRobot(robot.id)" :class="{ active: robot.status === 'returning' }">
                  복귀 명령
                </button>
                <button v-if="robot.status === 'active'" class="action-btn emergency" @click="emergencyStop(robot.id)" :class="{ active: robot.status === 'emergency' }">
                비상 정지
                </button>
                <button v-else class="action-btn active" @click="emergencyStop(robot.id)" :class="{ active: robot.status === 'active' }">
                  가동 시작
                </button>
                  
              </div>
              <!-- 상세 페이지 버튼 -->
              <button class="detail-btn" @click="goToDetailPage(robot.id)">
                상세 페이지
              </button>
            </div>
            <!-- 로봇 관리 버튼 -->
          <div class="robot-status-card robot-management" @click="robotsStore.openRobotManagementModal">
            <div class="status-details">
              <div class="status-item">
                <div class="plus-icon">+</div>
                <button class="manage-button">로봇 관리</button>
              </div>
            </div>
        </div>
      </div>
    </div>
  </div>

    <!-- 로봇 관리 모달 -->
    <div v-if="robotsStore.showRobotManagementModal" class="modal-overlay">
      <div class="modal-container management">
        <div class="modal-header">
          <h3>로봇 관리</h3>
          <div class="modal-buttons">
            <button @click="robotsStore.openAddRobotModal" class="add-robot-btn">로봇 등록</button>
            <button @click="robotsStore.closeRobotManagementModal" class="close-btn">닫기</button>
          </div>
        </div>

        <table class="robot-table">
          <thead>
            <tr>
              <th>ID</th>
              <th>이름</th>
              <th>IP 주소</th>
              <th>배터리</th>
              <th>위치</th>
              <th>상태</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="robot in robots" :key="robot.id">
              <td>{{ robot.id }}</td>
              <td>{{ robot.nickname || robot.name }}</td>
              <td>{{ robot.ipAddress }}</td>
              <td>{{ robot.battery }}%</td>
              <td>{{ robot.location }}</td>
              <td>
                <span :class="getStatusClass(robot.status)">{{ getStatusLabel(robot.status) }}</span>
              </td>
              <td>
                <button v-if="robot.status !== 'breakdown'" class="action-btn breakdown" @click="setBreakdown(robot.id)">고장</button>
                <button v-else class="action-btn active" @click="setActive(robot.id)">가동</button>
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>

    <!-- 로봇 등록 모달 -->
    <div v-if="robotsStore.showModal" class="modal-overlay">
      <div class="modal-container add-robot">
        <h3>로봇 등록하기</h3>
        <label>로봇명:</label>
        <input v-model="robotsStore.newRobot.name" placeholder="로봇 이름 입력" />
        <br>
        <label>IP주소: </label>
        <input v-model="robotsStore.newRobot.ipAddress" placeholder="로봇 IP 주소 입력" />
        <div class="modal-buttons">
          <button @click="robotsStore.handleAddRobot">등록</button>
          <button @click="robotsStore.closeModal" class="close-btn">닫기</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useRouter } from 'vue-router';
import { webSocketService } from '@/services/websocket'
import { useRobotsStore } from '@/stores/robots'

// 모니터링 컴포넌트 순서 관리
const router = useRouter();
const robotsStore = useRobotsStore()
const robots = computed(() => robotsStore.registered_robots)
const hiddenRobots = ref([]) // 숨겨진 로봇 ID 저장

// 숨김 처리된 로봇을 제외한 목록 반환
const visibleRobots = computed(() => {
  return robots.value.filter(robot => !hiddenRobots.value.includes(robot.id))
})

// 특정 로봇을 UI에서만 숨김 처리
const hideRobot = (robotId) => {
  hiddenRobots.value.push(robotId)
}

// 상태 클래스 주기
const getStatusClass = (status) => {
  return `status-badge ${status}`
}

const setBreakdown = (robotId) => {
  robotsStore.setBreakdown(robotId)
}

const setActive = (robotId) => {
  robotsStore.setActive(robotId)
}

// 상태 레이블
const getStatusLabel = (status) => {
  const labels = {
    active: '활동 중',
    charging: '충전 중',
    stopped: '정지 중',
    error: '오류 발생',
    idle: '대기 중',
    returning: '복귀 중',
    breakdown: '고장'
  }
  return labels[status] || status
}

// 배터리 색상 반환
const getBatteryColor = (battery) => {
  battery = Number(battery);
  if (battery >= 80) return 'green';
  if (battery >= 50) return 'yellowgreen';
  if (battery >= 30) return 'orange';
  if (battery >= 15) return 'orangered';
  return 'red';
}

// 로봇 제어 함수
// 복귀 명령 처리
const returnRobot = async (robotId) => {
  if (!robotId) return;
  try {
    const robotIndex = robotsStore.registered_robots.findIndex((r) => r.id === robotId);
    if (robotIndex !== -1) {
      robotsStore.registered_robots[robotIndex].status = 'returning'; // 상태 변경
    }
    console.log('복귀 명령:', robotId);
  } catch (err) {
    console.error('로봇 복귀 명령 에러:', err);
  }
};

// 비상 정지 및 가동 시작 처리
const emergencyStop = async (robotId) => {
  if (!robotId) return;
  try {
    const robotIndex = robotsStore.registered_robots.findIndex((r) => r.id === robotId);
    if (robotIndex !== -1) {
      const robot = robotsStore.registered_robots[robotIndex];
      robot.status = robot.status === 'active' ? 'stopped' : 'active';
    }
  } catch (err) {
    console.error('비상 정지 명령 에러:', err);
  }
};

// 상세 페이지 이동 함수
const goToDetailPage = (robotId) => {
  router.push(`/${robotId}`);
};

// WebSocket 연결 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/ws');

    // 로봇 상태 구독
    webSocketService.subscribe('monitoring/robots', (data) => {
      console.log('로봇 상태 업데이트:', data);
      loadRobotsWithNicknames(); // 상태 업데이트 후 닉네임 유지
    });

  } catch (error) {
    console.error('WebSocket 연결 실패:', error);
  }
};

// 로봇 데이터 불러오기 및 닉네임 설정
const loadRobotsWithNicknames = async () => {
  robots.value = robotsStore.registered_robots.map(robot => {
    return {
      ...robot,
      nickname: localStorage.getItem(`robot_nickname_${robot.id}`) || robot.name
    };
  });
};

onMounted(() => {
  // robotsStore.loadRobots();
  loadRobotsWithNicknames(); // 닉네임 불러오기
  setupWebSocket();
});

onUnmounted(() => {
  if (webSocketService.isConnected()) {
    webSocketService.disconnect()
  }
})
</script>

<style scoped>
.robot-management {
  display: flex;
  flex-direction: column;
  align-items: center;
  height: 89%; /* 카드 전체 높이를 활용 */
  text-align: center;
}

.plus-icon {
  font-size: 150px;
  color: rgb(234, 231, 231);
}

.manage-button {
  padding: 7px 10px;
  border: none;
  background-color: rgb(234, 231, 231);
  border-radius: 5px;
  cursor: pointer;
}

.realtime-monitoring {
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 5px 20px;
  padding: 0 20px;
}

.monitoring-sections {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 20px;
  overflow-y: auto;
}

.monitoring-section {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.section-header {
  display: flex;
  align-items: center;
  padding: 5px 15px;
  border-bottom: 1px solid #eee;
}

.section-header h4 {
  margin: 0;
  font-size: 16px;
  color: #333;
}

.robot-status-list {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
  padding: 20px;
}

.robot-status-card {
  position: relative;
  background: white;
  border-radius: 8px;
  padding: 20px;
  border: 1px solid #eee;
}

.robot-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1rem;
}

.robot-name {
  font-weight: 500;
}

.status-badge {
  padding: 0.25rem 0.5rem;
  border-radius: 999px;
  font-size: 0.75rem;
  font-weight: 500;
  display: inline-block;
  text-align: center;
  min-width: 60px;
}

.status-badge.active {
  background-color: #28a745;
  color: white;
}

.status-badge.charging {
  background-color: #007bff;
  color: white;
}

.status-badge.stopped {
  background-color: #dc3545;
  color: white;
}

.status-badge.error {
  background-color: #dc3545;
  color: white;
}

.status-badge.idle {
  background-color: #6c757d;
  color: white;
}

.status-badge.returning {
  background-color: black;
  color: white;
}

.status-badge.unknown {
  background-color: #ffc107;
  color: black;
}

.status-badge.breakdown {
  background-color: black;
  color: white;
}

.status-details {
  display: flex;
  flex-direction: column;
  gap: 15px;
  margin: 15px 0;
}

.status-item {
  margin-bottom: 0.5rem;
}

.status-item .label {
  display: block;
  color: #666;
  font-size: 0.875rem;
  margin-bottom: 0.25rem;
}

.battery-indicator {
  width: 100%;
  height: 20px;
  background: #f0f0f0;
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
  right: 10px;
  top: 50%;
  transform: translateY(-50%);
  color: white;
  font-size: 12px;
  text-shadow: 0 0 2px rgba(0, 0, 0, 0.5);
}

.action-btn {
  flex: 1;
  padding: 8px;
  border: none;
  border-radius: 4px;
  background: #007bff;
  color: white;
  cursor: pointer;
  transition: background-color 0.3s;
}

.action-btn:hover:not(:disabled) {
  background: #0056b3;
}

.robot-actions {
  display: flex;
  gap: 10px;
  padding: 5px 15px;
  background: #f8f9fa;
  margin-top: 20px;
}

/* 로봇 상태 변경 버튼 */
.robot-actions .action-btn {
  flex: 1;
  padding: 10px;
  border: none;
  border-radius: 5px;
  font-size: 14px;
  cursor: pointer;
  color: white;
}

/* 상세 페이지 버튼 스타일 */
.detail-btn {
  width: calc(100% - 30px); /* 기존 버튼 2개 넓이와 동일 */
  margin: 5px 15px;
  padding: 8px;
  background-color: black;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 1em;
  text-align: center;
}

.detail-btn:hover {
  background-color: #3c3c3dd1;
}

.action-btn.return {
  background: grey;
  color: white;
}

.action-btn.return:hover {
  background: rgb(153, 147, 147);
}

.action-btn.emergency {
  background: #F44336;
  color: white;
}

.action-btn.emergency:hover {
  background: #D32F2F;
}

/* 모달 스타일 */
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 999;  /* 다른 요소보다 위에 배치 */
}

.modal-container {
  background: white;
  padding: 20px;
  border-radius: 10px;
  width: 600px;
}

/* 모달 스타일 */
.modal-container.add-robot {
  background: white;
  padding: 20px;
  border-radius: 10px;
  width: 400px;
  text-align: left;
  box-shadow: 0px 4px 6px rgba(0, 0, 0, 0.1);
}

/* 입력 필드 스타일 */
.modal-container input {
  width: calc(100% - 20px);
  padding: 10px;
  margin: 5px 0 15px;
  border: 1px solid #ccc;
  border-radius: 5px;
  font-size: 14px;
}

.modal-buttons button {
  padding: 10px 15px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-size: 14px;
}

.modal-buttons .close-btn {
  background: #dc3545;
  color: white;
}

.modal-buttons .close-btn:hover {
  background: #c82333;
}

/* 고장 버튼 */
.action-btn.breakdown {
  background: #dc3545; /* 빨간색 */
  color: white;
}

.action-btn.breakdown:hover {
  background: #c82333;
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 15px;
}

.modal-buttons {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
}

.add-robot-btn {
  background: #28a745;
  color: white;
  padding: 5px 10px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
}

/* 테이블 스타일 */
.robot-table {
  width: 100%;
  border-collapse: collapse;
}

.robot-table th, .robot-table td {
  border: 1px solid #ddd;
  padding: 10px;
  text-align: center;
}

.robot-table th {
  background: #f4f4f4;
}

.status-badge {
  padding: 5px 10px;
  border-radius: 999px;
  font-size: 0.875rem;
  font-weight: 500;
  display: inline-block;
}

.close-btnx {
  position: absolute;
  top: 3px;
  right: 3px;
  background: none;
  border: none;
  font-size: 16px;
  cursor: pointer;
  color: #888;
}

.close-btnx:hover {
  color: #333;
}
</style>