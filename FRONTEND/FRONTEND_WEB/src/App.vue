<template>
  <div id="app" class="fixed inset-0 overflow-hidden">
    <div class="flex w-full h-full overflow-hidden">
      <!-- 메인 컨텐츠 영역 -->
      <div class="flex flex-col flex-1 overflow-hidden">
        <!-- Navbar (로그인 페이지에서만 숨김) -->
        <nav 
          v-if="!isLoginPage" 
          class="bg-gradient-to-b from-blue-200 to-blue-50 px-10 h-[55px] flex items-center justify-between"
        >
          <div class="cursor-pointer" @click="refreshPage">
            <img src="@/assets/logo.png" alt="로고" class="h-14">
          </div>
          <div class="flex space-x-8">
            <router-link to="/" class="text-gray-800 font-bold text-lg hover:text-blue-600">현황</router-link>
            <router-link to="/camera" class="text-gray-800 font-bold text-lg hover:text-blue-600">CCTV</router-link>
            <router-link to="/control" class="text-gray-800 font-bold text-lg hover:text-blue-600">제어</router-link>
            <router-link to="/enrollment" class="text-gray-800 font-bold text-lg hover:text-blue-600">등록</router-link>
            <router-link to="/statistics" class="text-gray-800 font-bold text-lg hover:text-blue-600">통계</router-link>
            <router-link to="/settings" class="text-gray-800 font-bold text-lg hover:text-blue-600">설정</router-link>
          </div>
        </nav>

        <div class="flex-1 px-5 overflow-hidden">
          <!-- Dynamic Page Content -->
          <div class="h-full overflow-hidden">
            <router-view />
          </div>
        </div>
      </div>

      <!-- Sidebar Section (로그인 페이지에서 숨김) -->
      <div v-if="!isLoginPage" class="w-[400px] bg-white border-l border-gray-300 overflow-hidden">
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
/* 스크롤바 제거 */
* {
  scrollbar-width: none; /* Firefox */
  -ms-overflow-style: none; /* IE and Edge */
}

*::-webkit-scrollbar {
  display: none;
}
</style>
