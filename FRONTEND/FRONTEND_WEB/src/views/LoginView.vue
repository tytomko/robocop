<template>
  <div class="login-container">
    <div class="left-column">
      <img src="@/assets/logo.png" alt="Robocop Logo" class="logo">
      <div class="slogan">
        <h2>세상에 없던</h2>
        <h2>자율 무인 경비로봇 시스템</h2>
        <h2>ROBOCOP이 만들어갑니다!</h2>
      </div>
    </div>
    <form @submit.prevent="handleLogin">
      <div class="right-column">
        <div class="login-form">
          <h3>아이디</h3>
          <input 
          v-model="loginForm.username" 
          type="text" 
          placeholder="ID를 입력해주세요">
          <h3>비밀번호</h3>
          <input 
          v-model="loginForm.password" 
          type="password"
          placeholder="비밀번호">
          <button type="submit" class="login-button">로그인</button>
        </div>
      </div>
    </form>
  </div>
</template>

<script setup>
import { ref } from 'vue';
import { useRouter } from 'vue-router';
import axios from 'axios';

const router = useRouter()
const loginForm = ref({
  username: '',
  password: '',
})

const handleLogin = async () => {
  try {
    // URLSearchParams 사용
    const params = new URLSearchParams();
    params.append('username', loginForm.value.username);
    params.append('password', loginForm.value.password);

    const response = await axios.post('https://robocop-backend-app.fly.dev/api/v1/auth/login', 
      params, 
      {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded'
        }
      }
    );

    const { accessToken, refreshToken } = response.data;

    // 토큰 저장
    localStorage.setItem('accessToken', accessToken);
    localStorage.setItem('refreshToken', refreshToken);

    // 로그인 성공 시 대시보드로 이동
    router.push({ name: 'monitoring' });
  } catch (error) {
    console.error('로그인 실패:', error);
    alert('로그인에 실패했습니다. 다시 시도해주세요.');
  }
};
</script>

<style scoped>
.login-container {
  display: flex;
  height: 100vh;
  width: 100%;
}

.left-column {
  width: 50%;
  background-color: #f5f5f5;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 2rem;
}

.logo {
  width: 200px;
  margin-bottom: 2rem;
}

.slogan {
  text-align: center;
  color: #333;
}

.slogan h2 {
  margin-bottom: 0.5rem;
  font-size: 1.5rem;
}

.right-column {
  width: 50%;
  min-width: 500px;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 2rem;
  height: 89vh; /* 높이를 화면 전체로 설정 */
}

.login-form {
  width: 80%;
  max-width: 400px;
}

.login-form h3 {
  margin-bottom: 0.5rem;
  color: #333;
}

.login-form input {
  width: 93%;
  padding: 0.8rem;
  margin-bottom: 1rem;
  border: 1px solid #ddd;
  border-radius: 4px;
}

.login-button {
  width: 100%;
  padding: 1rem;
  background-color: #000;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 1rem;
}

.login-button:hover {
  background-color: #333;
}

</style> 