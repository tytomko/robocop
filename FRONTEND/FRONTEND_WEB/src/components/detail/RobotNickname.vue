<template>
    <div class="settings-modal">
      <div class="modal-content">
        <div class="modal-header">
          <h3>로봇 설정</h3>
        </div>
        <label>로봇명: </label>
        <input v-model="robot.nickname" placeholder="로봇명을 설정하세요" />
  
        <div class="modal-buttons">
          <button @click="saveNickname">저장</button>
          <button @click="closeModal" class="close-btn">닫기</button>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
const props = defineProps({
  show: Boolean,
  robot: Object
});

const emit = defineEmits(['save', 'close']);

const saveNickname = () => {
  emit('save', props.robot.id, props.robot.nickname); // 부모 컴포넌트로 데이터 전달
};

const closeModal = () => {
  emit('close'); // 모달 닫기 이벤트
};
  </script>
  
  <style scoped>
  .settings-modal {
    position: fixed;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    background: rgba(0, 0, 0, 0.5);
    display: flex;
    justify-content: center;
    align-items: center;
    z-index: 1000;
  }
  
  .modal-content {
    background: white;
    padding: 20px;
    border-radius: 10px;
    width: 400px;
    text-align: left;
    box-shadow: 0px 4px 6px rgba(0, 0, 0, 0.1);
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

  /* 입력 필드 스타일 */
.modal-content input {
  width: calc(100% - 20px);
  padding: 10px;
  margin: 5px 0 15px;
  border: 1px solid #ccc;
  border-radius: 5px;
  font-size: 14px;
}
  </style>