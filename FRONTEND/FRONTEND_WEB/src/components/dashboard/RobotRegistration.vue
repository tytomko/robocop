<template>
  <div v-if="show" class="modal-overlay">
    <div class="modal-container add-robot">
      <h3>로봇 등록하기</h3>
      <label>로봇명:</label>
      <input v-model="newRobot.name" placeholder="로봇 이름 입력" />
      <br>
      <label>IP주소: </label>
      <input v-model="newRobot.ipAddress" placeholder="로봇 IP 주소 입력" />
      <div class="modal-buttons">
        <button @click="$emit('handleAddRobot')">등록</button>
        <button @click="$emit('close')" class="close-btn">닫기</button>
      </div>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  show: Boolean,
  newRobot: Object
});

const emit = defineEmits(['close', 'handleAddRobot']);
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
  z-index: 999;
}

.modal-container {
  background: white;
  padding: 20px;
  border-radius: 10px;
  width: 400px;
  text-align: left;
  box-shadow: 0px 4px 6px rgba(0, 0, 0, 0.1);
}

.modal-container input {
  width: calc(100% - 20px);
  padding: 10px;
  margin: 5px 0 15px;
  border: 1px solid #ccc;
  border-radius: 5px;
  font-size: 14px;
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
</style>