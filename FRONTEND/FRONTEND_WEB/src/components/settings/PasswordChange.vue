<template>
    <div v-if="isOpen" class="modal-overlay">
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
          <button @click="changePassword" :disabled="saving">비밀번호 변경</button>
          <button @click="closeModal" class="cancel-button">취소</button>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue';
  import axios from 'axios';
  
  const props = defineProps({
    isOpen: Boolean,
  });
  
  const emit = defineEmits(['close']);
  
  const passwordForm = ref({
    currentPassword: '',
    newPassword: '',
    confirmPassword: '',
  });
  
  const saving = ref(false);
  
  const changePassword = async () => {
    if (passwordForm.value.newPassword !== passwordForm.value.confirmPassword) {
      alert('새 비밀번호와 확인 비밀번호가 일치하지 않습니다.');
      return;
    }
  
    saving.value = true;
    try {
      await axios.post(
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
      closeModal();
    } catch (error) {
      console.error('비밀번호 변경 실패:', error.response?.data || error.message);
      alert(error.response?.data?.message || '비밀번호 변경에 실패했습니다. 다시 시도해주세요.');
    } finally {
      saving.value = false;
    }
  };
  
  const closeModal = () => {
    emit('close');
  };
  </script>
  
  <style scoped>
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
    border-radius: 12px;
    border: 1px solid #ddd;
    background-color: #007BFF;
    color: white;
    font-size: 16px;
    cursor: pointer;
  }
  
  .modal-actions button:hover {
    background-color: #0056b3;
  }
  
  .cancel-button {
    background: #ccc;
    border: none;
    padding: 10px 20px;
    border-radius: 5px;
    cursor: pointer;
  }

  /* 라벨과 입력 필드를 수평으로 배치 */
.modal-content .form-group {
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 15px;
}

.modal-content label {
  font-weight: bold;
  margin-right: 10px;
  flex-basis: 30%;
}

.modal-content input {
  width: 65%;
  padding: 10px;
  border-radius: 5px;
  border: 1px solid #ccc;
  font-size: 16px;
}
  </style>
  