<template>
  <div id="app">
    <div class="app-container">
      <!-- 메인 컨텐츠 영역 -->
      <div class="content-area">
        <!-- Navbar (로그인 페이지에서만 숨김) -->
        <nav class="navbar" v-if="!isLoginPage">
          <div class="navbar-container">
            <div class="logo-area" @click="refreshPage">
              <img src="@/assets/logo.png" alt="로고" class="logo">
            </div>
            <div class="navbar-links">
              <router-link to="/" class="navbar-link">현황</router-link>
              <router-link to="/camera" class="navbar-link">CCTV</router-link>
              <router-link to="/control" class="navbar-link">제어</router-link>
              <router-link to="/enrollment" class="navbar-link">등록</router-link>
              <router-link to="/statistics" class="navbar-link">통계</router-link>
              <router-link to="/settings" class="navbar-link">설정</router-link>
            </div>
          </div>
        </nav>

        <div class="main-container">
          <!-- Dynamic Page Content -->
          <div class="content">
            <router-view />
          </div>
        </div>
      </div>

      <!-- Sidebar Section (별도 컨테이너) -->
      <div class="sidebar-area" v-if="!isLoginPage">
        <SidebarSection />
      </div>
    </div>
  </div>
</template>

<script>
import { useRoute } from "vue-router";
import { computed } from "vue";
import SidebarSection from "@/components/dashboard/SidebarSection.vue";

export default {
  components: {
    SidebarSection,
  },
  setup() {
    const route = useRoute();
    const isLoginPage = computed(() => route.path === "/login");

    // 새로고침 대신 localhost:3000으로 이동
    const refreshPage = () => {
      window.location.href = "http://localhost:3000";
    };

    return { isLoginPage, refreshPage };
  },
};
</script>

<style>
/* 모든 요소의 스크롤바 제거 */
* {
  scrollbar-width: none; /* Firefox */
  -ms-overflow-style: none; /* IE and Edge */
}

/* Webkit 브라우저용 스크롤바 제거 */
*::-webkit-scrollbar {
  display: none;
}
</style>

<style scoped>
/* 글로벌 스타일 */
body {
  margin: 0;
  padding: 0;
  font-family: Arial, sans-serif;
  overflow: hidden;
}

#app {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  overflow: hidden;
}

.app-container {
  display: flex;
  width: 100%;
  height: 100%;
  overflow: hidden;
}

.content-area {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.navbar {
  background: linear-gradient(to bottom, #b0c4de, #f0f8ff);
  padding: 5px 40px;
  height: 60px;
}

.navbar-container {
  display: flex;
  width: 100%;
  justify-content: space-between;
  align-items: center;
  height: 100%;
}

.logo-area img {
  height: 40px;
  cursor: pointer;
}

.navbar-links {
  display: flex;
  gap: 30px;
}

.navbar-link {
  color: #333;
  font-weight: bold;
  text-decoration: none;
  font-size: 17px;
}

.main-container {
  flex: 1;
  padding: 0 20px;
  overflow: hidden;
}

.content {
  height: 100%;
  overflow: hidden;
}

.sidebar-area {
  width: 400px;
  background: white;
  border-left: 1px solid #ddd;
  overflow: hidden;
}
</style>