<template>
  <div class="settings-container p-5 flex flex-col gap-5">
    <div class="page-header flex justify-between items-center">
      <h2 class="text-xl font-bold">설정</h2>
      <div class="header-actions flex gap-3">
        <button @click="isChangePasswordModalOpen = true" class="bg-blue-500 text-white px-4 py-2 rounded-lg hover:bg-blue-700">비밀번호 재설정</button>
        <button @click="logout" class="bg-red-500 text-white px-4 py-2 rounded-lg hover:bg-red-700">로그아웃</button>
        <button @click="saveSettings" class="bg-green-500 text-white px-4 py-2 rounded-lg hover:bg-green-700 disabled:bg-gray-400" :disabled="saving">
          <i class="fas fa-save"></i> 저장
        </button>
      </div>
    </div>

    <PasswordChange :isOpen="isChangePasswordModalOpen" @close="isChangePasswordModalOpen = false" />

    <div class="settings-grid grid grid-cols-1 md:grid-cols-2 gap-5 overflow-y-auto">
      <!-- 일반 설정 -->
      <div class="settings-card bg-white rounded-lg p-5 shadow-md">
        <h3 class="text-lg font-semibold mb-3">일반 설정</h3>
        <div class="settings-form flex flex-col gap-4">
          <div class="form-group flex justify-between items-center">
            <label class="text-gray-600">언어</label>
            <select v-model="settings.general.language" class="border rounded-lg p-2">
              <option value="ko">한국어</option>
              <option value="en">English</option>
            </select>
          </div>
          <div class="form-group flex justify-between items-center">
            <label class="text-gray-600">테마</label>
            <select v-model="settings.general.theme" class="border rounded-lg p-2">
              <option value="light">라이트</option>
              <option value="dark">다크</option>
              <option value="system">시스템</option>
            </select>
          </div>
          <div class="form-group flex justify-between items-center">
            <label class="text-gray-600">시간대</label>
            <select v-model="settings.general.timezone" class="border rounded-lg p-2">
              <option value="Asia/Seoul">서울 (UTC+9)</option>
              <option value="UTC">UTC</option>
            </select>
          </div>
        </div>
      </div>

      <!-- 알림 설정 -->
      <div class="settings-card bg-white rounded-lg p-5 shadow-md">
        <h3 class="text-lg font-semibold mb-3">알림 설정</h3>
        <div class="settings-form flex flex-col gap-4">
          <div class="form-group flex justify-between items-center">
            <label class="text-gray-600">알림 사용</label>
            <input type="checkbox" v-model="settings.notifications.enabled" class="toggle-input">
          </div>
        </div>
      </div>

      <!-- 데이터 설정 -->
      <div class="settings-card bg-white rounded-lg p-5 shadow-md">
        <h3 class="text-lg font-semibold mb-3">데이터 설정</h3>
        <div class="settings-form flex flex-col gap-4">
          <div class="form-group flex justify-between items-center">
            <label class="text-gray-600">데이터 저장 기간</label>
            <select v-model="settings.data.retentionPeriod" class="border rounded-lg p-2">
              <option value="30">30일</option>
              <option value="60">60일</option>
              <option value="90">90일</option>
              <option value="180">180일</option>
              <option value="365">1년</option>
            </select>
          </div>
        </div>
      </div>

      <!-- 보안 설정 -->
      <div class="settings-card bg-white rounded-lg p-5 shadow-md">
        <h3 class="text-lg font-semibold mb-3">보안 설정</h3>
        <div class="settings-form flex flex-col gap-4">
          <div class="form-group flex justify-between items-center">
            <label class="text-gray-600">2단계 인증</label>
            <input type="checkbox" v-model="settings.security.twoFactor" class="toggle-input">
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue';
import PasswordChange from '@/components/settings/PasswordChange.vue';
import { useUser } from '@/composables/useUser';

const saving = ref(false);
const isChangePasswordModalOpen = ref(false);
const { logout } = useUser();

const settings = ref({
  general: {
    language: 'ko',
    theme: 'light',
    timezone: 'Asia/Seoul'
  },
  notifications: {
    enabled: true
  },
  data: {
    retentionPeriod: '90'
  },
  security: {
    twoFactor: false
  }
});

const saveSettings = async () => {
  saving.value = true;
  try {
    await new Promise(resolve => setTimeout(resolve, 1000));
    console.log('설정 저장:', settings.value);
  } catch (error) {
    console.error('설정 저장 중 오류:', error);
  } finally {
    saving.value = false;
  }
};
</script>
