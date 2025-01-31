<template>
  <div class="dashboard-container">
    <!-- 모바일 햄버거 메뉴 -->
    <div class="mobile-menu-toggle" :class="{ 'open': isMobileMenuOpen }">
      <div class="hamburger" @click="toggleMobileMenu">
        <span></span>
        <span></span>
        <span></span>
      </div>
    </div>

    <!-- 모바일 메뉴 오버레이 -->
    <div class="mobile-overlay" v-if="isMobileMenuOpen" @click="toggleMobileMenu"></div>

    <!-- 왼쪽 메뉴 -->
    <div class="left-sidebar" :class="{ 'open': isMobileMenuOpen }">
      <div class="mobile-menu-close" @click="toggleMobileMenu">×</div>
      <div class="logo-area" @click="refreshPage" style="cursor: pointer;">
        <img src="@/assets/logo.png" alt="로고" class="logo">
      </div>
      <div class="user-info">
        <div v-if="companyName" class="company-name">{{ companyName }}</div>
        <div class="admin-name">{{ adminName }}</div>
      </div>
      <nav class="main-menu">
        <div
          v-for="item in menuItems"
          :key="item.id"
          class="menu-group"
        >
          <div 
            class="menu-item"
            :class="{ active: activeMenu === item.id }"
            @click="handleMenuClick(item)"
          >
            {{ item.name }}
          </div>
          <div 
            class="submenu-items"
            :class="{ expanded: activeMenu === item.id }"
          >
            <router-link
              v-for="subMenu in item.subMenus"
              :key="subMenu.id"
              :to="{ name: subMenu.id }"
              class="submenu-item"
              :class="{ active: $route.name === subMenu.id }"
            >
              {{ subMenu.name }}
            </router-link>
          </div>
        </div>
      </nav>
    </div>

    <!-- 메인 컨텐츠 영역 -->
    <div class="main-content">
      <router-view></router-view>
    </div>

    <!-- 오른쪽 실시간 모니터링 사이드바 -->
    <div class="right-sidebar">
      <draggable 
        v-model="sidebarComponents" 
        class="sidebar-sections"
        handle=".section-handle"
        item-key="id"
        @start="drag=true" 
        @end="drag=false"
      >
        <template #item="{element}">
          <div class="sidebar-section">
            <div class="section-header">
              <div class="section-handle">⋮⋮</div>
              <h3>{{ element.title }}</h3>
            </div>

            <!-- 실시간 알림 섹션 -->
            <div v-if="element.type === 'alerts'" class="alert-section">
              <div class="alert-list">
                <div class="alert-item warning">
                  <span class="time">오후 04:27</span>
                  <span class="message">Robot 2의 배터리가 부족합니다. (25%)</span>
                </div>
                <div class="alert-item info">
                  <span class="time">오후 04:22</span>
                  <span class="message">Robot 1이 순찰을 시작했습니다.</span>
                </div>
              </div>
            </div>

            <!-- 실시간 모니터링 섹션 -->
            <div v-if="element.type === 'monitoring'" class="video-container">
              <div class="robot-selector">
                <select v-model="selectedRobot" class="robot-select" @change="handleRobotSelection">
                  <option value="">로봇 선택</option>
                  <option v-for="robot in robots" :key="robot.id" :value="robot.id">
                    {{ robot.name }}
                  </option>
                </select>
              </div>
              <div v-if="selectedRobot" class="camera-sections">
                <RobotCameras 
                  :robotId="selectedRobot" 
                  v-if="selectedRobot"
                />
              </div>
              <div v-else class="no-robot-selected">
                로봇을 선택해주세요
              </div>
            </div>

            <!-- 상태 정보 섹션 -->
            <div v-if="element.type === 'info'" class="monitoring-info">
              <div class="robot-selector">
                <select v-model="selectedRobot" class="robot-select" @change="handleRobotSelection">
                  <option value="">로봇 선택</option>
                  <option v-for="robot in robots" :key="robot.id" :value="robot.id">
                    {{ robot.name }}
                  </option>
                </select>
              </div>
              <div v-if="selectedRobot" class="info-content">
                <div class="info-item">
                  <span class="label">현재 상태:</span>
                  <span class="value">정상 운영 중</span>
                </div>
                <div class="info-item">
                  <span class="label">배터리:</span>
                  <span class="value">85%</span>
                </div>
                <div class="info-item">
                  <span class="label">현재 위치:</span>
                  <span class="value">1층 로비</span>
                </div>
              </div>
              <div v-else class="no-robot-selected">
                로봇을 선택해주세요
              </div>
            </div>
          </div>
        </template>
      </draggable>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, watch } from 'vue'
import { useRouter, useRoute } from 'vue-router'
import { useCompanyStore } from '@/stores/company'
import CameraView from '@/components/dashboard/monitoring/CameraView.vue'
import draggable from 'vuedraggable'
import PointCloudViewer from '@/components/dashboard/monitoring/PointCloudViewer.vue'
import RobotCameras from '@/components/dashboard/monitoring/RobotCameras.vue'

const router = useRouter()
const route = useRoute()
const activeMenu = ref('')
const loading = ref(true)
const error = ref(null)
const companyStore = useCompanyStore()

// companyName과 adminName을 store에서 가져옴
const companyName = computed(() => companyStore.companyName || '')
const adminName = computed(() => companyStore.adminName || '관리자')

const menuItems = [
  {
    id: 'monitoring',
    name: '현황',
    subMenus: [
      { id: 'monitoring', name: '실시간 모니터링' },
      { id: 'point-cloud', name: '라이다 정보' },
      { id: 'timeline', name: '타임라인' },
      { id: 'map', name: '맵 뷰' }
    ]
  },
  {
    id: 'robot',
    name: '로봇 관리',
    subMenus: [
      { id: 'robot-management', name: '로봇 관리' },
      { id: 'robot-control', name: '로봇 제어' },
      { id: 'course-management', name: '경로 관리' },
      { id: 'sensor-data', name: '센서 데이터' }
    ]
  },
  {
    id: 'stats',
    name: '통계',
    subMenus: [
      { id: 'stats-summary', name: '통계 요약' },
      { id: 'stats-video', name: '영상 조회' }
    ]
  },
  {
    id: 'settings',
    name: '설정',
    subMenus: [
      { id: 'settings', name: '시스템 설정' }
    ]
  }
]

// 현재 라우트에 해당하는 메뉴 찾기
const findMenuByRoute = (routeName) => {
  for (const menu of menuItems) {
    const subMenu = menu.subMenus.find(sub => sub.id === routeName)
    if (subMenu) {
      return menu
    }
  }
  return null
}

// 메뉴 초기화
const initializeMenu = async () => {
  loading.value = true
  error.value = null
  try {
    await router.isReady()
    const currentRouteName = route.name
    
    if (currentRouteName) {
      const menu = findMenuByRoute(currentRouteName)
      if (menu) {
        activeMenu.value = menu.id
      } else if (currentRouteName !== 'monitoring') {
        await router.push({ name: 'monitoring' })
      }
    }
  } catch (err) {
    console.error('메뉴 초기화 에러:', err)
    error.value = '메뉴를 불러오는데 실패했습니다.'
  } finally {
    loading.value = false
  }
}

// 메뉴 클릭 핸들러
const handleMenuClick = async (item) => {
  if (!item || !item.id) return
  
  activeMenu.value = item.id
  
  // 모바일에서 메뉴 클릭시 사이드바 닫기
  if (window.innerWidth <= 768) {
    isMobileMenuOpen.value = false
  }
  
  // 해당 메뉴의 첫 번째 서브메뉴로 이동
  if (item.subMenus && item.subMenus.length > 0) {
    try {
      await router.push({ name: item.subMenus[0].id })
    } catch (err) {
      console.error('라우팅 에러:', err)
      error.value = '페이지 이동에 실패했습니다.'
    }
  }
}

// 로봇 선택 처리
const selectedRobot = ref('');  // 선택된 로봇 ID
const handleRobotSelection = (event) => {
  selectedRobot.value = event.target.value;
};

// 사이드바 컴포넌트 순서 관리
const sidebarComponents = ref([
  { id: 1, type: 'alerts', title: '실시간 알림' },
  { id: 2, type: 'monitoring', title: '실시간 모니터링' },
  { id: 3, type: 'info', title: '상태 정보' }
])

// 라우트 변경 감지
watch(() => route.name, (newRouteName) => {
  if (newRouteName) {
    const menu = findMenuByRoute(newRouteName)
    if (menu) {
      activeMenu.value = menu.id
    }
  }
}, { immediate: true })

onMounted(async () => {
  await initializeMenu()
  
  // 저장된 로봇 선택 불러오기
  const savedRobot = localStorage.getItem('selectedRobot')
  if (savedRobot) {
    selectedRobot.value = savedRobot
  }
})

const isMobileMenuOpen = ref(false)
const selectedRobotInfo = computed(() => selectedRobot.value)

// 임시 로봇 데이터
const robots = ref([
  { id: 'robot1', name: 'Robot 1' }  // robot1로 통일
])

const toggleMobileMenu = () => {
  isMobileMenuOpen.value = !isMobileMenuOpen.value
}

const refreshPage = () => {
  window.location.reload()
}
</script>

<style scoped>
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

.dashboard-container {
  display: flex;
  height: 100vh;
  width: 100%;
  overflow: hidden;
  background-color: #f5f5f5;
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
}

.left-sidebar {
  width: 280px;
  background: #1a1a1a;
  color: white;
  display: flex;
  flex-direction: column;
  flex-shrink: 0;
  overflow-y: auto;
}

.logo-area {
  padding: 20px;
  text-align: center;
  background: white;
}

.logo {
  height: 40px;
  width: auto;
}

.user-info {
  padding: 15px 20px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
  background: #1a1a1a;
}

.company-name {
  font-size: 15px;
  font-weight: 500;
  margin-bottom: 4px;
  color: #fff;
  line-height: 1.4;
}

.admin-name {
  font-size: 14px;
  color: rgba(255, 255, 255, 0.7);
  line-height: 1.4;
}

.main-menu {
  padding: 10px 0;
}

.menu-group {
  margin-bottom: 2px;
}

.menu-item {
  padding: 12px 20px;
  cursor: pointer;
  transition: all 0.3s;
  background: #1a1a1a;
}

.menu-item:hover {
  background-color: rgba(255, 255, 255, 0.1);
}

.menu-item.active {
  background-color: #007bff;
}

.submenu-items {
  max-height: 0;
  overflow: hidden;
  transition: max-height 0.3s ease-out;
  background: #2a2a2a;
}

.submenu-items.expanded {
  max-height: 200px; /* 서브메뉴 최대 높이 */
}

.submenu-item {
  display: block;
  padding: 10px 20px 10px 40px;
  color: #fff;
  text-decoration: none;
  transition: background-color 0.3s;
  font-size: 0.95em;
}

.submenu-item:hover {
  background-color: rgba(255, 255, 255, 0.1);
}

.submenu-item.active {
  background-color: rgba(0, 123, 255, 0.2);
  color: #fff;
}

.main-content {
  flex: 1;
  overflow: auto;
  padding: 20px;
  background: white;
  min-width: 0;
  display: flex;
  flex-direction: column;
}

.right-sidebar {
  width: 400px;
  background: white;
  border-left: 1px solid #ddd;
  display: flex;
  flex-direction: column;
  flex-shrink: 0;
  overflow-y: auto;
}

.monitoring-header {
  padding: 20px;
  border-bottom: 1px solid #ddd;
}

.monitoring-header h3 {
  margin: 0;
  color: #333;
}

.video-container {
  padding: 20px;
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.camera-section {
  background: #1a1a1a;
  border-radius: 8px;
  overflow: hidden;
  margin-bottom: 1rem;
}

.camera-section h3 {
  padding: 0.5rem 1rem;
  margin: 0;
  background: #2a2a2a;
  color: white;
}

.monitoring-info {
  padding: 20px;
  border-top: 1px solid #ddd;
}

.info-item {
  display: flex;
  justify-content: space-between;
  margin-bottom: 10px;
  padding: 8px;
  background: #f8f9fa;
  border-radius: 4px;
}

.info-item .label {
  color: #666;
}

.info-item .value {
  font-weight: 500;
  color: #333;
}

@media (max-width: 1600px) {
  .right-sidebar {
    width: 350px;
  }
}

@media (max-width: 768px) {
  .mobile-menu-toggle {
    display: block;
  }

  .mobile-overlay {
    display: block;
  }

  .mobile-menu-close {
    display: block;
  }

  .left-sidebar {
    position: fixed;
    left: -280px;
    top: 0;
    bottom: 0;
    z-index: 1000;
    transition: left 0.3s ease;
    box-shadow: 2px 0 4px rgba(0, 0, 0, 0.1);
  }

  .left-sidebar.open {
    left: 0;
  }

  .right-sidebar {
    display: none;
  }

  .main-content {
    padding: 20px;
    padding-top: 60px;  /* 햄버거 메뉴 높이만큼 상단 여백 추가 */
  }
}

.alert-section {
  padding: 20px;
  background: white;
}

.alert-section h3 {
  margin: 0 0 15px 0;
  color: #333;
}

.alert-list {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.alert-item {
  padding: 12px;
  border-radius: 4px;
  display: flex;
  flex-direction: column;
  gap: 5px;
}

.alert-item.warning {
  background-color: #fff3cd;
  border: 1px solid #ffeeba;
  color: #856404;
}

.alert-item.info {
  background-color: #cce5ff;
  border: 1px solid #b8daff;
  color: #004085;
}

.alert-item .time {
  font-size: 0.85em;
  opacity: 0.8;
}

.alert-item .message {
  font-weight: 500;
}

.divider {
  height: 1px;
  background: #ddd;
  margin: 0;
}

.sidebar-sections {
  display: flex;
  flex-direction: column;
  gap: 1px;
  background: #f5f5f5;
  flex: 1;
}

.sidebar-section {
  background: white;
}

.section-header {
  display: flex;
  align-items: center;
  padding: 15px 20px;
  border-bottom: 1px solid #eee;
  background: white;
}

.section-handle {
  cursor: move;
  padding: 0 10px;
  color: #666;
  font-size: 18px;
  margin-right: 10px;
}

.section-header h3 {
  margin: 0;
  font-size: 16px;
  color: #333;
}

/* 모바일 메뉴 토글 버튼 */
.mobile-menu-toggle {
  display: none;
  position: fixed;
  top: 20px;
  left: 20px;
  z-index: 1002;
  cursor: pointer;
  background: white;
  padding: 10px;
  border-radius: 4px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.mobile-overlay {
  display: none;
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  z-index: 999;
}

.hamburger {
  width: 24px;
  height: 18px;
  position: relative;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
}

.hamburger span {
  display: block;
  width: 100%;
  height: 2px;
  background: #333;
  transition: all 0.3s ease;
}

/* 햄버거 메뉴 애니메이션 */
.mobile-menu-toggle.open .hamburger span:nth-child(1) {
  transform: translateY(8px) rotate(45deg);
}

.mobile-menu-toggle.open .hamburger span:nth-child(2) {
  opacity: 0;
}

.mobile-menu-toggle.open .hamburger span:nth-child(3) {
  transform: translateY(-8px) rotate(-45deg);
}

.mobile-menu-close {
  display: none;
  position: absolute;
  top: 10px;
  right: 10px;
  font-size: 24px;
  color: white;
  cursor: pointer;
  padding: 10px;
}

/* 로봇 선택 드롭다운 */
.robot-selector {
  padding: 15px 20px;
  border-bottom: 1px solid #eee;
}

.robot-select {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
  background: white;
  font-size: 14px;
}

.no-robot-selected {
  padding: 20px;
  text-align: center;
  color: #666;
}

/* 드롭다운 스타일 개선 */
.robot-select {
  appearance: none;
  background-image: url("data:image/svg+xml;charset=UTF-8,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3e%3cpolyline points='6 9 12 15 18 9'%3e%3c/polyline%3e%3c/svg%3e");
  background-repeat: no-repeat;
  background-position: right 8px center;
  background-size: 16px;
  padding-right: 32px;
}

.robot-select:focus {
  outline: none;
  border-color: #007bff;
}

.monitoring-section {
  margin: 20px;
}

.monitoring-container {
  display: grid;
  grid-template-columns: 1fr 1fr;  /* 2개의 동일한 크기의 열 */
  gap: 20px;
  margin-top: 20px;
}

.monitoring-item {
  background: white;
  border-radius: 8px;
  padding: 15px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

/* 반응형 디자인 */
@media (max-width: 1200px) {
  .monitoring-container {
    grid-template-columns: 1fr;  /* 화면이 작을 때는 한 열로 */
  }
}
</style> 