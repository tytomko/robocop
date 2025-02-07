<template>
    <div class="user-registration">
      <div class="header">
        <h1>등록된 사용자 명단</h1>
        <button v-if="users.length > 0" @click="openModal" class="header-button">등록하기</button>
      </div>
  
      <!-- ✅ 분리된 UserList 컴포넌트 사용 -->
      <UserList :users="users" :openModal="openModal" />
  
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
  import UserList from '@/components/enrollment/UserList.vue';
  
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
</style>
