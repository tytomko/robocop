<template>
  <div id="app" class="fixed inset-0 bg-gray-100 overflow-auto">
    <div class="flex w-full h-full">
      <!-- Left Sidebar -->
      <ListSidebarSection
        v-if="!isLoginPage"
        :is-collapsed="isLeftSidebarCollapsed"
        @toggle-left-sidebar="toggleLeftSidebar"
      />

      <!-- Main Content -->
      <div class="flex flex-col flex-1 overflow-auto">

        <!-- Navbar -->
        <nav v-if="!isLoginPage" class="bg-gray-900 text-white px-10 h-[55px] flex items-center justify-between">
          <div class="cursor-pointer" @click="refreshPage">
            <img src="@/assets/whitelogo.png" alt="로고" class="h-14" />
          </div>
          <div :class="navRouterLinkContainerClasses">
            <router-link to="/" class="font-normal text-base hover:text-blue-400">현황</router-link>
            <router-link to="/camera" class="font-normal text-base hover:text-blue-400">CCTV</router-link>
            <router-link to="/control" class="font-normal text-base hover:text-blue-400">제어</router-link>
            <router-link to="/statistics" class="font-normal text-base hover:text-blue-400">통계</router-link>
            <router-link to="/management" class="font-normal text-base hover:text-blue-400">관리</router-link>
            <AlarmNotification inline v-if="!isSidebarCollapsed" class="ml-4" />
          </div>
        </nav>

        <div class="flex-1 px-5 overflow-auto">
          <router-view />
        </div>
      </div>

      <!-- Right Sidebar -->
      <div v-if="!isLoginPage" :class="sidebarClasses">
        <MapSidebarSection :isCollapsed="isSidebarCollapsed" @toggle-sidebar="toggleSidebar" />
      </div>
    </div>

    <!-- 우측 상단 고정 알림 아이콘 -->
    <AlarmNotification v-if="!isLoginPage && isSidebarCollapsed" :isCollapsed="true" />
  </div>
</template>

<script setup>
import { computed, ref, watch } from "vue";
import { useRoute } from "vue-router";
import MapSidebarSection from "@/components/dashboard/MapSidebarSection.vue";
import AlarmNotification from "@/components/dashboard/AlarmNotification.vue";
import ListSidebarSection from "./components/dashboard/ListSidebarSection.vue";

const route = useRoute();
const isLoginPage = computed(() => route.path === "/login");

// 사이드바 상태 저장 및 토글
const storedState = localStorage.getItem("sidebar-collapsed");
const isSidebarCollapsed = ref(storedState ? storedState === "true" : false);

const toggleSidebar = () => {
  isSidebarCollapsed.value = !isSidebarCollapsed.value;
  localStorage.setItem("sidebar-collapsed", isSidebarCollapsed.value);
  setTimeout(() => {
    window.dispatchEvent(new Event("resize"));
  }, 350);
};

const storedLeftState = localStorage.getItem("left-sidebar-collapsed");
const isLeftSidebarCollapsed = ref(storedLeftState ? storedLeftState === "true" : false);

const toggleLeftSidebar = () => {
  isLeftSidebarCollapsed.value = !isLeftSidebarCollapsed.value;
  localStorage.setItem("left-sidebar-collapsed", isLeftSidebarCollapsed.value);
  setTimeout(() => {
    window.dispatchEvent(new Event("resize"));
  }, 350);
};

// 사이드바 클래스 설정
const sidebarClasses = computed(() => {
  return isSidebarCollapsed.value
    ? "w-0"
    : "w-[400px] bg-white border-l border-gray-300 relative";
});

// 네비게이션 링크 컨테이너 클래스
const navRouterLinkContainerClasses = computed(() => {
  return isSidebarCollapsed.value
    ? "flex items-center space-x-8 mr-20"
    : "flex items-center space-x-8 mr-0";
});

// 페이지 새로고침
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
/* 스크롤바 숨기되, 스크롤 가능하게 유지 */
* {
  scrollbar-width: none; /* Firefox */
  -ms-overflow-style: none; /* IE, Edge */
}
*::-webkit-scrollbar {
  display: none; /* Chrome, Safari */
}
</style>
