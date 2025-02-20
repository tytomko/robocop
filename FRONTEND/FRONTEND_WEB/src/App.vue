<template>
  <div id="app" class="fixed inset-0 bg-gray-100 overflow-auto">
    <div class="layout-container">
      <!-- Left Sidebar -->
      <ListSidebarSection
        v-if="!isLoginPage"
        :is-collapsed="isLeftSidebarCollapsed"
        @toggle-left-sidebar="toggleLeftSidebar"
        class="sidebar-container z-30"
      />

      <!-- Main Content -->
      <div class="flex flex-col flex-1 overflow-auto relative">
        <!-- Navbar -->
        <nav v-if="!isLoginPage" class="bg-gray-900 text-white px-4 lg:px-10 h-[55px] flex items-center justify-between sticky top-0 z-40">
          <!-- 로고 및 경보 시스템 -->
          <div class="flex items-center">
            <div class="cursor-pointer" @click="refreshPage">
              <img src="@/assets/whitelogo.png" alt="로고" class="h-10 lg:h-14" />
            </div>
            <AlertSystem /> <!-- AlertSystem 컴포넌트 추가 -->
          </div>

          <!-- 데스크탑 네비게이션 -->
          <div class="hidden lg:flex items-center space-x-8">
            <router-link 
              v-for="link in navLinks"
              :key="link.path"
              :to="link.path"
              class="nav-link"
              :class="{ 'nav-link-active': route.path === link.path }"
            >
              {{ link.name }}
            </router-link>
            <!-- 데스크탑 알림 - 사이드바 확장 시 -->
            <AlarmNotification inline class="ml-4"  />
          </div>

          <!-- 모바일 네비게이션 -->
          <div class="lg:hidden flex items-center space-x-4">
            <!-- 모바일 알림 아이콘 -->
            <AlarmNotification compact class="mr-2" />
            
            <!-- 햄버거 메뉴 버튼 -->
            <button @click="toggleMobileMenu" class="p-2" aria-label="메뉴 열기">
              <svg class="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path 
                  stroke-linecap="round" 
                  stroke-linejoin="round" 
                  stroke-width="2"
                  d="M4 6h16M4 12h16M4 18h16" 
                />
              </svg>
            </button>

            <!-- 모바일 드롭다운 메뉴 -->
            <div 
              v-if="mobileMenuOpen" 
              class="absolute right-0 top-[55px] bg-gray-900 w-48 rounded-b-lg shadow-lg z-40"
            >
              <router-link 
                v-for="link in navLinks"
                :key="link.path"
                :to="link.path"
                class="mobile-nav-link"
                :class="{ 'bg-white/15': route.path === link.path }"
                @click="closeMobileMenu"
              >
                {{ link.name }}
              </router-link>
            </div>
          </div>
        </nav>

        <!-- 메인 콘텐츠 영역 -->
        <div class="main-content z-10">
          <router-view />
        </div>
      </div>

      <!-- Right Sidebar -->
      <!-- <div 
        v-if="!isLoginPage" 
        :class="[
          sidebarClasses,
          'sidebar-container z-30'
        ]"
      >
        <MapSidebarSection 
          :isCollapsed="isSidebarCollapsed" 
          @toggle-sidebar="toggleSidebar" 
        />
      </div> -->
    </div>

    <!-- 고정 알림 아이콘 (데스크탑 사이드바 축소 시) -->
    <!-- <AlarmNotification 
      v-if="!isLoginPage && isSidebarCollapsed" 
      :isCollapsed="true"
      class="hidden lg:block fixed right-2 z-35"
      :class="{'top-[55px]': !isSidebarCollapsed, 'top-[10px]': isSidebarCollapsed}"
    /> -->
  </div>
</template>

<script setup>
// 스크립트 부분은 이전과 동일하게 유지
import { computed, ref, watch, onMounted, onUnmounted } from "vue";
import { useRoute } from "vue-router";
// import MapSidebarSection from "@/components/dashboard/MapSidebarSection.vue";
import AlarmNotification from "@/components/dashboard/AlarmNotification.vue";
import ListSidebarSection from "./components/dashboard/ListSidebarSection.vue";
import AlertSystem from "./components/ai/AlertSystem.vue";
import { webSocketService } from '@/services/websocket';
import { useRobotsStore } from '@/stores/robots';

const robotsStore = useRobotsStore();
const route = useRoute();
const isLoginPage = computed(() => route.path === "/login");

// 네비게이션 링크 데이터
const navLinks = [
  { path: '/', name: '현황' },
  { path: '/camera', name: 'CCTV' },
  { path: '/control', name: '제어' },
  { path: '/management', name: '관리' }
];

// 사이드바 상태 관리
const storedState = localStorage.getItem("sidebar-collapsed");
const isSidebarCollapsed = ref(storedState ? storedState === "true" : false);

const storedLeftState = localStorage.getItem("left-sidebar-collapsed");
const isLeftSidebarCollapsed = ref(storedLeftState ? storedLeftState === "true" : false);

const toggleLeftSidebar = () => {
  if (window.innerWidth >= 1024) {
    isLeftSidebarCollapsed.value = !isLeftSidebarCollapsed.value;
    localStorage.setItem("left-sidebar-collapsed", isLeftSidebarCollapsed.value);
    // store 상태 업데이트
    robotsStore.updateSidebarStates(isLeftSidebarCollapsed.value, isSidebarCollapsed.value);
    setTimeout(() => window.dispatchEvent(new Event("resize")), 350);
  }
};

// 모바일 메뉴 상태 관리
const mobileMenuOpen = ref(false);
const toggleMobileMenu = () => {
  mobileMenuOpen.value = !mobileMenuOpen.value;
};
const closeMobileMenu = () => {
  mobileMenuOpen.value = false;
};

// 페이지 새로고침
const refreshPage = () => {
  window.location.href = "https://robocopbackendssafy.duckdns.org/";
};

// 라우트 변경 감지
watch(
  () => route.path,
  () => {
    closeMobileMenu();
    setTimeout(() => window.dispatchEvent(new Event("resize")), 350);
  }
);

onMounted(async () => {
  try {
    await webSocketService.connect('wss://robocopbackendssafy.duckdns.org/ws/test');
    robotsStore.setWebSocketConnected(true);
    console.log('웹소켓 연결 성공');
  } catch (error) {
    console.error('WebSocket 연결 실패:', error);
    robotsStore.setWebSocketConnected(false);
  }
});

onUnmounted(() => {
  webSocketService.disconnect();
});

// 사이드바 토글 함수
// const toggleSidebar = () => {
//   if (window.innerWidth >= 1024) {
//     isSidebarCollapsed.value = !isSidebarCollapsed.value;
//     localStorage.setItem("sidebar-collapsed", isSidebarCollapsed.value);
//     // store 상태 업데이트
//     robotsStore.updateSidebarStates(isLeftSidebarCollapsed.value, isSidebarCollapsed.value);
//     setTimeout(() => window.dispatchEvent(new Event("resize")), 350);
//   }
// };
// 사이드바 클래스
// const sidebarClasses = computed(() => {
//   return isSidebarCollapsed.value
//     ? "w-0"
//     : "w-[400px] bg-white border-l border-gray-300 relative";
// });
</script>
