<template>
  <div id="app" class="fixed inset-0 overflow-hidden bg-gray-100">
    <div class="flex w-full h-full overflow-hidden">
      <!-- Main Content 영역 -->
      <div class="flex flex-col flex-1 overflow-hidden">
        <!-- Navbar: 로고는 좌측, router-link들은 오른쪽에 정렬 -->
        <nav v-if="!isLoginPage" class="bg-gray-900 text-white px-10 h-[55px] flex items-center justify-between" style="font-family: 'Noto Sans KR', sans-serif;">
          <!-- 로고 영역 -->
          <div class="cursor-pointer" @click="refreshPage">
            <img src="@/assets/whitelogo.png" alt="로고" class="h-14" />
          </div>
          <!-- router-link 및 inline AlarmNotification -->
          <div :class="navRouterLinkContainerClasses">
            <router-link to="/" class="font-normal text-base hover:text-blue-400">현황</router-link>
            <router-link to="/camera" class="font-normal text-base hover:text-blue-400">CCTV</router-link>
            <router-link to="/control" class="font-normal text-base hover:text-blue-400">제어</router-link>
            <router-link to="/enrollment" class="font-normal text-base hover:text-blue-400">등록</router-link>
            <router-link to="/statistics" class="font-normal text-base hover:text-blue-400">통계</router-link>
            <router-link to="/settings" class="font-normal text-base hover:text-blue-400">설정</router-link>
            <!-- 사이드바 확장 시: inline 모드로 AlarmNotification 렌더링 -->
            <AlarmNotification inline v-if="!isSidebarCollapsed" class="ml-4" />
          </div>
        </nav>

        <div class="flex-1 px-5 overflow-hidden">
          <!-- Dynamic Page Content -->
          <div class="h-full overflow-hidden">
            <router-view />
          </div>
        </div>
      </div>

      <!-- Sidebar 컨테이너 (로그인 페이지가 아니라면 항상 렌더링) -->
      <div v-if="!isLoginPage" :class="sidebarClasses">
        <SidebarSection :isCollapsed="isSidebarCollapsed" @toggle-sidebar="toggleSidebar" />
      </div>
    </div>
    <!-- 사이드바 축소 시: 기본 fixed 모드의 AlarmNotification -->
    <div v-if="!isLoginPage && isSidebarCollapsed">
      <AlarmNotification />
    </div>
  </div>
</template>

<script setup>
import { computed, ref, watch } from "vue";
import { useRoute } from "vue-router";
import SidebarSection from "@/components/dashboard/SidebarSection.vue";
import AlarmNotification from "@/components/dashboard/AlarmNotification.vue";

const route = useRoute();
const isLoginPage = computed(() => route.path === "/login");

// localStorage에서 사이드바 상태를 읽어 초기 상태 설정 (없으면 false)
const storedState = localStorage.getItem("sidebar-collapsed");
const isSidebarCollapsed = ref(storedState ? storedState === "true" : false);

const toggleSidebar = () => {
  isSidebarCollapsed.value = !isSidebarCollapsed.value;
  localStorage.setItem("sidebar-collapsed", isSidebarCollapsed.value);
  setTimeout(() => {
    window.dispatchEvent(new Event("resize"));
  }, 350);
};

// 사이드바 컨테이너: 확장 시 400px, 축소 시 0px (메인 콘텐츠가 전체 너비 사용)
const sidebarClasses = computed(() => {
  return isSidebarCollapsed.value
    ? "w-0"
    : "w-[400px] bg-white border-l border-gray-300 relative";
});

// router-link들을 감싸는 컨테이너 클래스: 사이드바 확장 상태에 따라 오른쪽 여백을 다르게 적용
const navRouterLinkContainerClasses = computed(() => {
  return isSidebarCollapsed.value
    ? "flex items-center space-x-8 mr-20"
    : "flex items-center space-x-8 mr-0";
});

const refreshPage = () => {
  window.location.href = "http://localhost:3000";
};

watch(
  () => route.path,
  () => {
    setTimeout(() => {
      window.dispatchEvent(new Event("resize"));
    }, 350);
  }
);
</script>

<style>
/* 스크롤바 제거 */
* {
  scrollbar-width: none;
  -ms-overflow-style: none;
}
*::-webkit-scrollbar {
  display: none;
}
</style>