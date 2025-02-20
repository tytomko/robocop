<template>
  <!-- 최상단 컨테이너 -->
  <div
    id="app"
    :class="[
      // 기존 배경이나 전체 화면 설정
      'fixed inset-0 bg-gray-100',
      // 로그인 페이지면 스크롤 막기
      { 'overflow-hidden': isLoginPage, 'overflow-auto': !isLoginPage }
    ]"
  >
    <!-- 로그인 페이지가 아닐 때만 기존 레이아웃을 보여줌 -->
    <template v-if="!isLoginPage">
      <div class="layout-container">
        <!-- Left Sidebar -->
        <ListSidebarSection
          :is-collapsed="isLeftSidebarCollapsed"
          @toggle-left-sidebar="toggleLeftSidebar"
          class="sidebar-container z-30"
        />

        <!-- 메인 영역 -->
        <div class="flex flex-col flex-1 relative">
          <!-- 상단 네비게이션 -->
          <nav class="bg-gray-900 text-white px-4 lg:px-10 h-[55px] flex items-center justify-between sticky top-0 z-40">
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

          <!-- 메인 콘텐츠 -->
          <div class="main-content z-10 overflow-auto">
            <router-view />
          </div>
        </div>

        <!-- (우측 사이드바가 있다면 필요에 따라 추가) -->
        <!--
        <div
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
        </div>
        -->
      </div>
    </template>

    <!-- 로그인 페이지일 때는 오직 LoginView만 렌더링 -->
    <template v-else>
      <router-view />
    </template>
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
import { useNotificationsStore } from "./stores/notifications";
import { useRobotsStore } from '@/stores/robots';

const robotsStore = useRobotsStore();
const notificationsStore = useNotificationsStore();
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

// const alertSSEConnections = new Map();

// onMounted(async () => {
//   // 로봇별 alert SSE 연결 설정
//   const setupAlertSSE = (seq) => {
//     const alertSSE = new EventSource(
//       `https://robocopbackendssafy.duckdns.org/api/v1/robots/sse/${seq}/alert`
//     );

//     alertSSE.onmessage = (event) => {
//       const data = JSON.parse(event.data);
//       if (data.alert) {
//         const { type, message } = data.alert;
//         console.log('Alert received:', type, message);  // 디버깅용 로그 추가
        
//         // emergency 타입일 경우 경보 시스템 활성화
//         if (type === 'emergency') {
//           robotsStore.alerts = true;
//         }
//         // clear 타입일 경우 경보 해제 및 알림 추가
//         else if (type === 'clear') {
//           robotsStore.alerts = false;
//           notificationsStore.addNotification({
//             message: '신원이 확인되었습니다',
//             timestamp: new Date().toISOString()
//           });
//         }
//         // caution 타입일 경우 알림 추가
//         else if (type === 'caution') {
//           notificationsStore.addNotification({
//             message: '거수자를 발견하였습니다',
//             timestamp: new Date().toISOString()
//           });
//         }
//       }
//     };

//     return alertSSE;
//   };

//   // seq=1로 고정하여 테스트
//   alertSSEConnections.set(1, setupAlertSSE(1));
// });

// onUnmounted(() => {
//   webSocketService.disconnect();
  
//   // alert SSE 연결 정리
//   alertSSEConnections.forEach(sse => sse.close());
//   alertSSEConnections.clear();
// });

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
