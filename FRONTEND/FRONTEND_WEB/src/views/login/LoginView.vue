<template>
  <div class="relative h-screen w-full">
    <!-- 배경 동영상 -->
    <video autoplay loop muted playsinline class="absolute inset-0 w-full h-full object-cover">
      <source src="@/assets/digital.mp4" type="video/mp4">
      브라우저가 동영상을 지원하지 않습니다.
    </video>

    <!-- Overlay (어두운 배경 효과) -->
    <div class="absolute inset-0 bg-black opacity-30"></div>

    <div class="flex h-screen w-full relative">
      <!-- Left Column -->
      <div class="w-1/2 flex flex-col items-center justify-center bg-gray-100 p-8 bg-opacity-80">
        <img src="@/assets/logo.png" alt="Robocop Logo" class="w-48 mb-6">
        <div class="text-center text-gray-800">
          <h2 class="text-2xl font-bold mb-1">세상에 없던</h2>
          <h2 class="text-2xl font-bold mb-1">자율 무인 경비로봇 시스템</h2>
          <h2 class="text-2xl font-bold">ROBOCOP이 만들어갑니다!</h2>
        </div>
      </div>

      <!-- Right Column -->
      <form @submit.prevent="handleLogin" class="w-1/2 flex items-center justify-center p-8 relative z-10">
        <div class="w-full max-w-md bg-white p-8 rounded-lg shadow-md bg-opacity-90">
          <h3 class="text-lg font-semibold text-gray-700 mb-2">아이디</h3>
          <input 
            v-model="loginForm.username" 
            type="text" 
            placeholder="ID를 입력해주세요" 
            class="w-full p-3 mb-4 border border-gray-300 rounded focus:outline-none focus:ring-2 focus:ring-gray-500">
          
          <h3 class="text-lg font-semibold text-gray-700 mb-2">비밀번호</h3>
          <input 
            v-model="loginForm.password" 
            type="password" 
            placeholder="비밀번호" 
            class="w-full p-3 mb-4 border border-gray-300 rounded focus:outline-none focus:ring-2 focus:ring-gray-500">
          
          <button type="submit" class="w-full p-3 bg-black text-white rounded hover:bg-gray-700">로그인</button>
        </div>
      </form>
    </div>
  </div>
</template>


<script setup>
import { ref } from 'vue';
import { useRouter } from 'vue-router';
import axios from 'axios';

const router = useRouter();
const loginForm = ref({
  username: '',
  password: '',
});

const handleLogin = async () => {
  try {
    const params = new URLSearchParams();
    params.append('username', loginForm.value.username);
    params.append('password', loginForm.value.password);

    const response = await axios.post('https://robocop-backend-app.fly.dev/api/v1/auth/login', params, {
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    });

    const { accessToken, refreshToken } = response.data;
    localStorage.setItem('accessToken', accessToken);
    localStorage.setItem('refreshToken', refreshToken);
    router.push({ name: 'monitoring' });
  } catch (error) {
    console.error('로그인 실패:', error);
    alert('로그인에 실패했습니다. 다시 시도해주세요.');
  }
};
</script>
