<template>
  <div v-if="isOpen" class="fixed inset-0 flex items-center justify-center bg-black bg-opacity-50 z-50">
    <div class="bg-white p-6 rounded-lg w-96 shadow-lg">
      <h3 class="text-xl font-semibold mb-4">비밀번호 변경</h3>
      
      <div class="mb-3 flex flex-col">
        <label class="font-medium mb-1">현재 비밀번호</label>
        <input type="password" v-model="passwordForm.currentPassword" placeholder="현재 비밀번호"
          class="w-full p-2 border rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500">
      </div>
      
      <div class="mb-3 flex flex-col">
        <label class="font-medium mb-1">새 비밀번호</label>
        <input type="password" v-model="passwordForm.newPassword" placeholder="새 비밀번호"
          class="w-full p-2 border rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500">
      </div>
      
      <div class="mb-3 flex flex-col">
        <label class="font-medium mb-1">새 비밀번호 확인</label>
        <input type="password" v-model="passwordForm.confirmPassword" placeholder="새 비밀번호 확인"
          class="w-full p-2 border rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500">
      </div>
      
      <div class="flex justify-between mt-4">
        <button @click="changePassword" :disabled="saving"
          class="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 disabled:opacity-50">
          비밀번호 변경
        </button>
        <button @click="closeModal" class="px-4 py-2 bg-gray-400 text-white rounded-lg hover:bg-gray-500">
          취소
        </button>
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
      'https://robocopbackendssafy.duckdns.org/api/v1/auth/change-password',
      {
        currentPassword: passwordForm.value.currentPassword,
        newPassword: passwordForm.value.newPassword,
        confirmPassword: passwordForm.value.confirmPassword,
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
