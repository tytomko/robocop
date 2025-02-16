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
            <router-link 
              to="/" 
              class="nav-link"
              :class="{ 'nav-link-active': route.path === '/' }"
            >
              현황
            </router-link>
            <router-link 
              to="/camera" 
              class="nav-link"
              :class="{ 'nav-link-active': route.path === '/camera' }"
            >
              CCTV
            </router-link>
            <router-link 
              to="/control" 
              class="nav-link"
              :class="{ 'nav-link-active': route.path === '/control' }"
            >
              제어
            </router-link>
            <router-link 
              to="/management" 
              class="nav-link"
              :class="{ 'nav-link-active': route.path === '/management' }"
            >
              관리
            </router-link>
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
import { computed, ref, watch, onMounted, onUnmounted } from "vue";
import { useRoute } from "vue-router";
import MapSidebarSection from "@/components/dashboard/MapSidebarSection.vue";
import AlarmNotification from "@/components/dashboard/AlarmNotification.vue";
import ListSidebarSection from "./components/dashboard/ListSidebarSection.vue";
import { webSocketService } from '@/services/websocket'
import { useRobotsStore } from '@/stores/robots'

const robotsStore = useRobotsStore()


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
  window.location.href = "https://robocopbackendssafy.duckdns.org/";
};

watch(
  () => route.path,
  () => {
    setTimeout(() => {
      window.dispatchEvent(new Event("resize"));
    }, 350);
  }
);

onMounted(async () => {
  try {
    await webSocketService.connect('wss://robocopbackendssafy.duckdns.org/ws/test')
    robotsStore.setWebSocketConnected(true)
    console.log('웹소켓 연결 성공')
  } catch (error) {
    console.error('WebSocket 연결 실패:', error)
    robotsStore.setWebSocketConnected(false)
  }
})

onUnmounted(() => {
  webSocketService.disconnect()
})

</script>

<style>
@import url('https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@300;400;500;700&display=swap');

/* 기본 폰트 설정 */
#app {
  font-family: 'Noto Sans KR', sans-serif;
}

/* 스크롤바 숨기기 */
* {
  scrollbar-width: none;
  -ms-overflow-style: none;
}

*::-webkit-scrollbar {
  display: none;
}

/* 네비게이션 링크 스타일 */
.nav-link {
  font-size: 1rem;
  font-weight: 500;
  letter-spacing: 0.025em;
  padding: 0.5rem 1rem;
  border-radius: 0.375rem;
  transition: all 0.2s ease;
  color: rgba(255, 255, 255, 0.9);
}

.nav-link:hover {
  background-color: rgba(255, 255, 255, 0.1);
  color: white;
}

.nav-link-active {
  background-color: rgba(255, 255, 255, 0.15);
  color: white;
  font-weight: 600;
}

/* 전역 타이포그래피 스타일 */
h1 { font-size: 1.875rem; font-weight: 700; }
h2 { font-size: 1.5rem; font-weight: 700; }
h3 { font-size: 1.25rem; font-weight: 600; }
h4 { font-size: 1.125rem; font-weight: 600; }
</style>