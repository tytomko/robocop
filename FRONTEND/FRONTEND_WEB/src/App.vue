<template>
  <div id="app" class="fixed inset-0 overflow-hidden bg-gray-100">
    <div class="flex w-full h-full overflow-hidden">
      <!-- Left Sidebar -->
      <!-- [1] 왼쪽 사이드바 -->
      <ListSidebarSection
        v-if="!isLoginPage"
        :is-collapsed="isLeftSidebarCollapsed"
        @toggle-left-sidebar="toggleLeftSidebar"
      />

      <!-- Main Content -->
      <div class="flex flex-col flex-1 overflow-hidden">

        <!-- Navbar -->
        <nav v-if="!isLoginPage" class="bg-gray-900 text-white px-10 h-[55px] flex items-center justify-between">
          <div class="cursor-pointer" @click="refreshPage">
            <img src="@/assets/whitelogo.png" alt="로고" class="h-14" />
          </div>
          <div :class="navRouterLinkContainerClasses">
            <router-link to="/" class="font-normal text-base hover:text-blue-400">현황</router-link>
            <router-link to="/camera" class="font-normal text-base hover:text-blue-400">CCTV</router-link>
            <router-link to="/control" class="font-normal text-base hover:text-blue-400">제어</router-link>
            <router-link to="/enrollment" class="font-normal text-base hover:text-blue-400">등록</router-link>
            <router-link to="/statistics" class="font-normal text-base hover:text-blue-400">통계</router-link>
            <router-link to="/settings" class="font-normal text-base hover:text-blue-400">설정</router-link>
            <!-- 사이드바 확장 상태일 때만 네비게이션 바에서 알림 아이콘 표시 -->
            <AlarmNotification inline v-if="!isSidebarCollapsed" class="ml-4" />
          </div>
        </nav>

        <div class="flex-1 px-5 overflow-hidden">
          <router-view />
        </div>
      </div>

      <!-- Right Sidebar -->
      <div v-if="!isLoginPage" :class="sidebarClasses">
        <MapSidebarSection :isCollapsed="isSidebarCollapsed" @toggle-sidebar="toggleSidebar" />
      </div>
    </div>

    <!-- 사이드바 축소 상태일 때만 우측 상단에 `fixed`로 알림 아이콘 표시 -->
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

// localStorage에서 사이드바 상태를 읽어 초기 상태 설정
const storedState = localStorage.getItem("sidebar-collapsed");
const isSidebarCollapsed = ref(storedState ? storedState === "true" : false);

const toggleSidebar = () => {
  isSidebarCollapsed.value = !isSidebarCollapsed.value;
  localStorage.setItem("sidebar-collapsed", isSidebarCollapsed.value);
  setTimeout(() => {
    window.dispatchEvent(new Event("resize"));
  }, 350);
};

// [왼쪽 사이드바] 접기/펼치기 상태
const storedLeftState = localStorage.getItem("left-sidebar-collapsed");
const isLeftSidebarCollapsed = ref(storedLeftState ? storedLeftState === "true" : false);

// [왼쪽 사이드바] 토글 함수
const toggleLeftSidebar = () => {
  isLeftSidebarCollapsed.value = !isLeftSidebarCollapsed.value;
  localStorage.setItem("left-sidebar-collapsed", isLeftSidebarCollapsed.value);
  // 크기 변경 이후 차트나 지도 등이 리사이즈되도록 resize 이벤트 트리거
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

// router-link 컨테이너의 여백 조정 (사이드바 확장/축소 상태 반영)
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