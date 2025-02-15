<template>
  <div class="settings-container p-5 flex flex-col gap-5 max-w-6xl mx-auto h-screen overflow-y-auto">
    
    <!-- ✅ 계정 관리 섹션 -->
    <div class="bg-white p-5 rounded-lg shadow-md">
      <div class="page-header flex justify-between items-center">
        <h2 class="text-xl font-bold">계정 관리</h2>
        <div class="header-actions flex gap-3">
          <button @click="isChangePasswordModalOpen = true" class="bg-blue-500 text-white px-4 py-2 rounded-lg hover:bg-blue-700">
            비밀번호 재설정
          </button>
          <button @click="logout" class="bg-red-500 text-white px-4 py-2 rounded-lg hover:bg-red-700">
            로그아웃
          </button>
        </div>
      </div>
      
      <!-- 비밀번호 변경 모달 -->
      <PasswordChange :isOpen="isChangePasswordModalOpen" @close="isChangePasswordModalOpen = false" />
    </div>

    <!-- ✅ 사용자 관리 섹션 -->
    <div class="bg-white p-5 rounded-lg shadow-md">
      <h2 class="text-xl font-bold mb-4">사용자 관리</h2>
      <EnrollmentView />
    </div>

  </div>
</template>

<script setup>
import { ref } from 'vue';
import PasswordChange from '@/components/management/PasswordChange.vue';
import EnrollmentView from '@/views/enrollment/EnrollmentView.vue';
import { useUser } from '@/composables/useUser';

const isChangePasswordModalOpen = ref(false);
const { logout } = useUser();
</script>
