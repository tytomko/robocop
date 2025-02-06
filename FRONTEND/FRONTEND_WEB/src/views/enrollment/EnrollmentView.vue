<template>
    <div class="user-registration">
      <div class="header">
        <h1>등록된 사용자 명단</h1>
        <button v-if="users.length > 0" @click="openModal" class="header-button">등록하기</button>
      </div>
  
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
  
      <EnrollModal
        :showModal="showModal"
        :formData="formData"
        @closeModal="closeModal"
        @submitForm="addUser"
      />
    </div>
  </template>
  
  <script setup>
  import { ref, reactive } from 'vue';
  import EnrollModal from '@/components/enrollment/EnrollModal.vue';  // 추가
  
  // 사용자 목록 (초기에는 빈 배열)
  const users = ref([]);
  
  // 모달 표시 여부
  const showModal = ref(false);
  
  // 폼 데이터 (이미지 파일은 여러 개 첨부 가능)
  const formData = reactive({
    name: '',
    birthDate: '',
    phone: '',
    images: [],
  });
  
  // 모달 열기
  const openModal = () => {
    showModal.value = true;
  };
  
  // 폼 데이터 리셋
  const resetForm = () => {
    formData.name = '';
    formData.birthDate = '';
    formData.phone = '';
    formData.images = [];
  };
  
  // 모달 닫기
  const closeModal = () => {
    showModal.value = false;
    resetForm();
  };
  
  // 사용자 추가
  const addUser = (newUser) => {
    users.value.push(newUser);
    closeModal();
  };
  </script>

<style scoped>
.user-registration {
padding: 20px;
font-family: sans-serif;
}

/* 헤더 영역: 제목과 등록하기 버튼을 같은 행에 배치 */
.header {
display: flex;
align-items: center;
justify-content: space-between;
margin-bottom: 20px;
}

.header h1 {
margin: 0;
}

.header-button {
padding: 8px 16px;
cursor: pointer;
}

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
