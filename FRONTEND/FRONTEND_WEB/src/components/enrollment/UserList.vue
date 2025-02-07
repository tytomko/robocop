<template>
    <div class="card-grid">
      <div v-if="users.length === 0" class="user-card add-user-card" @click="openModal">
        <div class="user-add-icon">+</div>
        <div class="user-add-text">등록하기</div>
      </div>
      <div v-for="user in users" :key="user.id" class="user-card">
        <img v-if="user.image" :src="user.image" alt="User Image" class="user-image" />
        <div v-else class="user-placeholder">No Image</div>
        <div class="user-name">{{ user.name }}</div>
        <div class="user-details">
          <div class="user-birthdate"><strong>생년월일: </strong>{{ user.birthDate }}</div>
          <div class="user-phone"><strong>휴대폰 번호: </strong>{{ user.phone }}</div>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  defineProps({
    users: Array,       // 사용자 리스트
    openModal: Function // 등록하기 버튼 클릭 시 실행할 함수
  });
  </script>
  
  <style scoped>
  /* 카드 그리드: 한 줄에 최대 3개 카드 */
  .card-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 20px;
    margin-bottom: 20px;
  }
  
  /* 사용자 카드 기본 스타일 */
  .user-card {
    border: 1px solid #ddd;
    border-radius: 8px;
    overflow: hidden;
    text-align: center;
    padding: 10px;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  }
  
  /* 사용자 이미지 */
  .user-image {
    width: 100%;
    height: 300px;
    object-fit: cover;
  }
  
  /* 이미지 없을 경우 플레이스홀더 */
  .user-placeholder {
    width: 100%;
    height: 300px;
    background: #f0f0f0;
    display: flex;
    align-items: center;
    justify-content: center;
    color: #999;
  }
  
  /* 사용자 이름 */
  .user-name {
    margin-top: 15px;
    font-size: 16px;
    font-weight: bold;
  }
  
  /* 사용자 상세 정보 */
  .user-details {
    margin-top: 8px;
    font-size: 14px;
    color: #333;
  }
  
  /* 등록하기 카드 (빈 카드) */
  .add-user-card {
    width: 93%;
    height: 350px;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    cursor: pointer;
  }
  
  .user-add-icon {
    font-size: 48px;
    color: #007BFF;
  }
  
  .user-add-text {
    margin-top: 10px;
    font-size: 18px;
    color: #007BFF;
    font-weight: bold;
  }
  </style>
  