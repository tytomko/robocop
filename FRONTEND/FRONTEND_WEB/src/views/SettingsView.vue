<template>
  <div class="settings-container">
    <div class="page-header">
      <h2>설정</h2>
      <div class="header-actions">
        <button @click="openChangePasswordModal" class="password-reset">비밀번호 재설정</button>
        <button @click="logout" class="logout">로그아웃</button>
        <button @click="saveSettings" class="save-button" :disabled="saving">
          <i class="fas fa-save"></i> 저장
        </button>
      </div>
    </div>
    
    <div class="settings-grid">
      <!-- 일반 설정 -->
      <div class="settings-card">
        <h3>일반 설정</h3>
        <div class="settings-form">
          <div class="form-group">
            <label>언어</label>
            <select v-model="settings.general.language">
              <option value="ko">한국어</option>
              <option value="en">English</option>
            </select>
          </div>
          <div class="form-group">
            <label>테마</label>
            <select v-model="settings.general.theme">
              <option value="light">라이트</option>
              <option value="dark">다크</option>
              <option value="system">시스템</option>
            </select>
          </div>
          <div class="form-group">
            <label>시간대</label>
            <select v-model="settings.general.timezone">
              <option value="Asia/Seoul">서울 (UTC+9)</option>
              <option value="UTC">UTC</option>
            </select>
          </div>
        </div>
      </div>

      <!-- 알림 설정 -->
      <div class="settings-card">
        <h3>알림 설정</h3>
        <div class="settings-form">
          <div class="form-group">
            <label>알림 사용</label>
            <div class="toggle-switch">
              <input
                type="checkbox"
                v-model="settings.notifications.enabled"
                :id="'notifications-enabled'"
              />
              <label :for="'notifications-enabled'"></label>
            </div>
          </div>
          <div class="form-group" v-if="settings.notifications.enabled">
            <label>이메일 알림</label>
            <div class="toggle-switch">
              <input
                type="checkbox"
                v-model="settings.notifications.email"
                :id="'notifications-email'"
              />
              <label :for="'notifications-email'"></label>
            </div>
          </div>
          <div class="form-group" v-if="settings.notifications.enabled">
            <label>푸시 알림</label>
            <div class="toggle-switch">
              <input
                type="checkbox"
                v-model="settings.notifications.push"
                :id="'notifications-push'"
              />
              <label :for="'notifications-push'"></label>
            </div>
          </div>
        </div>
      </div>

      <!-- 데이터 설정 -->
      <div class="settings-card">
        <h3>데이터 설정</h3>
        <div class="settings-form">
          <div class="form-group">
            <label>데이터 저장 기간</label>
            <select v-model="settings.data.retentionPeriod">
              <option value="30">30일</option>
              <option value="60">60일</option>
              <option value="90">90일</option>
              <option value="180">180일</option>
              <option value="365">1년</option>
            </select>
          </div>
          <div class="form-group">
            <label>자동 백업</label>
            <div class="toggle-switch">
              <input
                type="checkbox"
                v-model="settings.data.autoBackup"
                :id="'data-autobackup'"
              />
              <label :for="'data-autobackup'"></label>
            </div>
          </div>
          <div class="form-group" v-if="settings.data.autoBackup">
            <label>백업 주기</label>
            <select v-model="settings.data.backupInterval">
              <option value="daily">매일</option>
              <option value="weekly">매주</option>
              <option value="monthly">매월</option>
            </select>
          </div>
        </div>
      </div>

      <!-- 보안 설정 -->
      <div class="settings-card">
        <h3>보안 설정</h3>
        <div class="settings-form">
          <div class="form-group">
            <label>2단계 인증</label>
            <div class="toggle-switch">
              <input
                type="checkbox"
                v-model="settings.security.twoFactor"
                :id="'security-twofactor'"
              />
              <label :for="'security-twofactor'"></label>
            </div>
          </div>
          <div class="form-group">
            <label>세션 타임아웃</label>
            <select v-model="settings.security.sessionTimeout">
              <option value="15">15분</option>
              <option value="30">30분</option>
              <option value="60">1시간</option>
              <option value="120">2시간</option>
            </select>
          </div>
          <div class="form-group">
            <label>IP 접근 제한</label>
            <div class="toggle-switch">
              <input
                type="checkbox"
                v-model="settings.security.ipRestriction"
                :id="'security-iprestriction'"
              />
              <label :for="'security-iprestriction'"></label>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div v-if="saving" class="loading-overlay">
      <div class="loading-spinner"></div>
    </div>
  </div>

<!--비밀번호 변경 폼-->
  <div v-if="isChangePasswordModalOpen" class="modal-overlay">
    <div class="modal-content">
      <h3>비밀번호 변경</h3>
      <div class="form-group">
        <label>현재 비밀번호</label>
        <input type="password" v-model="passwordForm.currentPassword" placeholder="현재 비밀번호" />
      </div>
      <div class="form-group">
        <label>새 비밀번호</label>
        <input type="password" v-model="passwordForm.newPassword" placeholder="새 비밀번호" />
      </div>
      <div class="form-group">
        <label>새 비밀번호 확인</label>
        <input type="password" v-model="passwordForm.confirmPassword" placeholder="새 비밀번호 확인" />
      </div>
      <div class="modal-actions">
        <button @click="changePassword" :disabled="saving">
          <i class="fa" aria-hidden="true">비밀번호 변경</i> 
        </button>
        <button @click="closeChangePasswordModal" class="cancel-button">취소</button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { useRouter } from 'vue-router'
import axios from 'axios'

const router = useRouter();
const saving = ref(false);
const isChangePasswordModalOpen = ref(false);

// 설정 상태
const settings = ref({
  general: {
    language: 'ko',
    theme: 'light',
    timezone: 'Asia/Seoul'
  },
  notifications: {
    enabled: true,
    email: true,
    push: true
  },
  data: {
    retentionPeriod: '90',
    autoBackup: true,
    backupInterval: 'daily'
  },
  security: {
    twoFactor: false,
    sessionTimeout: '30',
    ipRestriction: false
  }
})

// 설정 저장
const saveSettings = async () => {
  saving.value = true
  try {
    // 실제 API 호출로 대체 필요
    await new Promise(resolve => setTimeout(resolve, 1000))
    console.log('설정 저장:', settings.value)
  } catch (error) {
    console.error('설정 저장 중 오류:', error)
  } finally {
    saving.value = false
  }
}

const logout = async () => {
  try {
    // 로그아웃 API 호출
    await axios.post('https://robocop-backend-app.fly.dev/api/v1/auth/logout', null, {
      headers: {
        Authorization: `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    // 클라이언트 측에서 토큰 제거
    localStorage.removeItem('accessToken'); // Access Token 삭제
    localStorage.removeItem('refreshToken'); // Refresh Token 삭제

    // Axios 기본 Authorization 헤더 제거
    delete axios.defaults.headers.common['Authorization'];

    // 사용자에게 알림 및 페이지 이동
    alert('로그아웃 되었습니다.');
    router.push('/login'); // 로그인 페이지로 리다이렉트
  } catch (error) {
    console.error('로그아웃 실패:', error);
    alert('로그아웃에 실패했습니다.');
  }
};

const passwordForm = ref({
  currentPassword: '',
  newPassword: '',
  confirmPassword: '',
});

const changePassword = async () => {
  // 비밀번호 확인 검증
  if (passwordForm.value.newPassword !== passwordForm.value.confirmPassword) {
    alert('새 비밀번호와 확인 비밀번호가 일치하지 않습니다.');
    return;
  }

  saving.value = true;
  try {
    // API 호출
    const response = await axios.post(
      'https://robocop-backend-app.fly.dev/api/v1/auth/change-password',
      {
        currentPassword: passwordForm.value.currentPassword,
        newPassword: passwordForm.value.newPassword,
        confirmPassword: passwordForm.value.confirmPassword
      },
      {
        headers: {
          Authorization: `Bearer ${localStorage.getItem('accessToken')}`,
        },
      }
    );

    alert('비밀번호가 성공적으로 변경되었습니다.');
    closeChangePasswordModal();

  } catch (error) {
  console.error('비밀번호 변경 실패:', error.response?.data || error.message);
  alert(error.response?.data?.message || '비밀번호 변경에 실패했습니다. 다시 시도해주세요.');
  } finally {
    saving.value = false;
  }
};

// 모달 열기
const openChangePasswordModal = () => {
  isChangePasswordModalOpen.value = true;
};

// 모달 닫기
const closeChangePasswordModal = () => {
  isChangePasswordModalOpen.value = false;
  resetPasswordForm();
};

// 비밀번호 입력 필드 초기화
const resetPasswordForm = () => {
  passwordForm.value.currentPassword = '';
  passwordForm.value.newPassword = '';
  passwordForm.value.confirmPassword = '';
};  

</script>

<style scoped>

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
  z-index: 1000;
}

.modal-content {
  background: white;
  padding: 20px;
  border-radius: 10px;
  width: 400px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.2);
  position: relative;
}

.modal-actions {
  display: flex;
  justify-content: space-between;
  margin-top: 20px;
}

.modal-actions button {
  padding: 10px 20px;
  border-radius: 12px;  /* 둥글게 설정 */
  border: 1px solid #ddd;
  background-color: #007BFF;
  color: white;
  font-size: 16px;
  cursor: pointer;
}

.modal-actions button:hover {
  background-color: #0056b3; /* hover 시 배경 색 */
}

/* 라벨과 입력 필드를 수평으로 배치 */
.modal-content .form-group {
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 15px; /* 각 입력창 간격 */
}

.modal-content label {
  font-weight: bold;
  margin-right: 10px; /* 라벨과 입력창 사이의 간격 */
  flex-basis: 30%; /* 라벨이 차지할 비율 */
}

.modal-content input {
  width: 65%; /* 입력 필드의 너비 */
  padding: 10px;
  border-radius: 5px;
  border: 1px solid #ccc;
  font-size: 16px;
}

.cancel-button {
  background: #ccc;
  border: none;
  padding: 10px 20px;
  border-radius: 5px;
  cursor: pointer;
}

.settings-container {
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

.settings-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
  overflow-y: auto;
}

.settings-card {
  background: white;
  border-radius: 8px;
  padding: 20px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.settings-card h3 {
  margin: 0 0 20px 0;
  color: #333;
  font-size: 18px;
}

.settings-form {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.form-group {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.form-group label {
  color: #666;
  font-size: 14px;
}

.form-group select {
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  min-width: 150px;
}

/* 토글 스위치 스타일 */
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
  border-radius: 34px;
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
  border-top: 3px solid #4CAF50;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

/* 버튼 기본 스타일 */
button {
  padding: 10px 20px;
  border-radius: 12px;  /* 둥글게 설정 */
  border: 1px solid #ddd;
  color: white;
  font-size: 16px;
  cursor: pointer;
  margin-right: 10px;  /* 버튼 간의 간격 */
}

/* 비밀번호 재설정 버튼 스타일 */
button.password-reset {
  background-color: #007BFF;
}

button.password-reset:hover {
  background-color: #0056b3;
}

button.password-reset:active {
  background-color: #004085;
}

/* 로그아웃 버튼 스타일 */
button.logout {
  background-color: #FF5733;
}

button.logout:hover {
  background-color: #e03d1f;
}

button.logout:active {
  background-color: #c6341a;
}

.save-button {
  padding: 10px 20px;
  border-radius: 12px;  /* 둥글게 설정 */
  border: 1px solid #ddd;
  background: #4CAF50;
  color: white;
  font-size: 16px;
  cursor: pointer;
  margin-right: 10px;  /* 버튼 간의 간격 */
}

.save-button:hover {
  background: #388E3C;
}

.save-button:disabled {
  background: #ccc;
  cursor: not-allowed;
}

</style> 