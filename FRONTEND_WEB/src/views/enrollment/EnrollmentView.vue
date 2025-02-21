<template>
  <div class="p-5 font-sans max-h-screen overflow-y-auto">
    <div class="flex items-center justify-between mb-5">
      <h1 class="text-2xl font-bold">등록된 사용자 명단</h1>
      <button
        v-if="users.length > 0"
        @click="openModal"
        class="px-4 py-2 bg-blue-500 text-white rounded-md shadow-md hover:bg-blue-600 transition"
      >
        등록하기
      </button>
    </div>

    <!-- 사용자 목록 표시 -->
    <UserList :users="users" :openModal="openModal" />

    <!-- 등록 모달 -->
    <EnrollModal
      :showModal="showModal"
      :formData="formData"
      @closeModal="closeModal"
      @submitForm="addUser"
    />
  </div>
</template>

<script setup>
import { ref, reactive, onMounted } from 'vue';
import EnrollModal from '@/components/enrollment/EnrollModal.vue';
import UserList from '@/components/enrollment/UserList.vue';
import axios from 'axios';

// 사용자 목록 (초기엔 빈 배열)
const users = ref([]);

// 모달 표시 여부
const showModal = ref(false);

// 폼 데이터 (이미지 파일은 여러 개 첨부 가능)
const formData = reactive({
  personName: '',
  personPosition: '',
  personPhone: '',
  images: [], // 여러 파일 객체를 저장
});

// 모달 열기/닫기, 폼 리셋 함수
const openModal = () => {
  showModal.value = true;
};

const resetForm = () => {
  formData.personName = '';
  formData.personPosition = '';
  formData.personPhone = '';
  formData.images = [];
};

const closeModal = () => {
  showModal.value = false;
  resetForm();
};

// 사용자 추가 함수 (axios, then/catch 사용)
const addUser = (newUser) => {
  if (!newUser.images || newUser.images.length === 0) {
    console.error("이미지 파일이 필요합니다.");
    return;
  }

  // 파일명 기준 오름차순 정렬 후 첫 번째 파일 선택
  const sortedImages = [...newUser.images].sort((a, b) =>
    a.name.localeCompare(b.name)
  );
  const selectedImage = sortedImages[0];

  // FormData 객체에 값을 추가 (파일 객체는 그대로 전송)
  const formDataToSend = new FormData();
  formDataToSend.append("personName", newUser.personName);
  formDataToSend.append("personPosition", newUser.personPosition);
  formDataToSend.append("image", selectedImage);
  formDataToSend.append("personPhone", newUser.personPhone);
  formDataToSend.append("personDepartment", newUser.personDepartment || "");

  // (디버깅용) FormData의 내용을 출력
  for (let [key, value] of formDataToSend.entries()) {
    console.log(key, value);
  }

  axios
    .post("https://robocop-backend-app.fly.dev/api/v1/persons", formDataToSend)
    .then((response) => {
      // API 응답에서 저장된 사용자 객체 추출 (필요 시 response.data.data로 조정)
      const savedUser = response.data;
      console.log("Saved user:", savedUser);
      users.value.push(savedUser);
      closeModal();
    })
    .catch((error) => {
      console.error("Error adding user:", error);
      closeModal();
    });
};

// GET API로 사용자 목록 불러오기
const getUsers = () => {
  axios
    .get('https://robocop-backend-app.fly.dev/api/v1/persons')
    .then((response) => {
      const resData = response.data;
      // 응답 구조: { success, status, message, timestamp, data: [ ... ] }
      if (resData && Array.isArray(resData.data)) {
        users.value = resData.data;
      } else {
        users.value = [];
      }
    })
    .catch((error) => {
      console.error("Error fetching users:", error);
      users.value = [];
    });
};

onMounted(() => {
  getUsers();
});
</script>
