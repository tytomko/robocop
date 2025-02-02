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
import { ref, onMounted } from 'vue';
import { useRouter } from 'vue-router';
import axios from 'axios';

const router = useRouter()
const loginForm = ref({
  username: '',
  password: '',
})

const handleLogin = async () => {
  try {
    // FormData 객체 생성
    const formData = new FormData();
    formData.append('username', loginForm.value.username);
    formData.append('password', loginForm.value.password);

    // URLSearchParams 사용
    const params = new URLSearchParams();
    params.append('username', loginForm.value.username);
    params.append('password', loginForm.value.password);

    const response = await axios.post('https://robocop-backend-app.fly.dev/api/v1/auth/login', 
      params,  // FormData 대신 URLSearchParams 사용
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
    console.log(accessToken, refreshToken)
  } catch (error) {
    console.error('로그인 실패:', error);
    alert('로그인에 실패했습니다. 다시 시도해주세요.');
  }
};
</script>

<style scoped>
body {
  margin: 0;
  padding: 0;
  height: 100%;
  overflow: hidden; /* 스크롤 방지 */
}

.login-container {
  display: flex;
  height: 100%; /* 부모 높이에 맞춤 */
  width: 100%; /* 전체 너비 차지 */
  position: relative;
  background-color: #ffffff;
  transition: all 0.3s ease;
}

.left-column {
  width: 50%;
  flex: 1 1 50%;
  min-width: 600px;
  background-color: #f5f5f5;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 2rem;
  transition: background-color 0.3s ease;
}

.logo {
  width: 200px;
  margin-bottom: 2rem;
}

.slogan {
  text-align: center;
  color: #333;
  transition: color 0.3s ease;
}

.slogan h2 {
  margin-bottom: 0.5rem;
  font-size: 1.5rem;
}

.right-column {
  width: 50%;
  flex: 1 1 50%;
  min-width: 600px;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 2rem;
  background-color: #ffffff;
  transition: background-color 0.3s ease;
  height: 89vh; /* 높이를 화면 전체로 설정 */
}

.login-form {
  width: 80%;
  max-width: 400px;
  margin: auto; /* 중앙 정렬 유지 */
}

.login-form h3 {
  margin-bottom: 0.5rem;
  color: #333;
  transition: color 0.3s ease;
}

.login-form input {
  width: 93%;
  padding: 0.8rem;
  margin-bottom: 1rem;
  border: 1px solid #ddd;
  border-radius: 4px;
  background-color: #ffffff;
  color: #333;
  transition: all 0.3s ease;
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
  transition: background-color 0.3s ease;
}

.login-button:hover {
  background-color: #333;
}

</style> 