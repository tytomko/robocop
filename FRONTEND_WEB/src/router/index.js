import { createRouter, createWebHistory } from 'vue-router'
import LoginView from '@/views/login/LoginView.vue'
import MonitoringView from '@/views/MonitoringView.vue'
import RobotDetailView from '@/views/robot/RobotDetailView.vue'
import CameraView from '@/views/camera/CameraView.vue'
import EnrollmentView from '@/views/enrollment/EnrollmentView.vue'
import RobotControlView from '@/views/robot/RobotControlView.vue'
import StatisticsView from '@/views/statistics/StatisticsView.vue'
import ManagementView from '@/views/management/ManagementView.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: 'monitoring',
      component: MonitoringView,
      meta: { requiresAuth: true } // 로그인 필요
    },
    {
      path: '/login',
      name: 'login',
      component: LoginView,
      meta: { guestOnly: true }
    },
    {
      path: '/management',
      name: 'management',
      component: ManagementView,
      meta: { requiresAuth: true } // 로그인 필요
    },
    {
      path: '/:seq',
      name: 'detail',
      component: RobotDetailView,
      meta: { requiresAuth: true } // 로그인 필요
    },
    {
      path: '/camera',
      name: 'camera',
      component: CameraView,
      meta: { requiresAuth: true } // 로그인 필요
    },
    {
      path: '/enrollment',
      name: 'enrollment',
      component: EnrollmentView,
      meta: { requiresAuth: true } // 로그인 필요
    },
    {
      path: '/control',
      name: 'control',
      component: RobotControlView,
      meta: { requiresAuth: true } // 로그인 필요
    },
    {
      path: '/statistics',
      name: 'statistics',
      component: StatisticsView,
      meta: { requiresAuth: true } // 로그인 필요
    }
  ]
})

// 네비게이션 가드 추가 (로그인 여부 확인)
router.beforeEach((to, from, next) => {
  const isAuthenticated = !!localStorage.getItem('accessToken'); // 로그인 상태 확인

  if (to.meta.requiresAuth && !isAuthenticated) {
    // 로그인 안 된 상태에서 인증이 필요한 페이지로 가려고 하면 로그인 페이지로 이동
    alert('로그인이 필요합니다.');
    next('/login');
  } else if (to.meta.guestOnly && isAuthenticated) {
    // 로그인된 사용자가 로그인 페이지(`/login`)에 접근하려고 하면 차단
    alert('이미 로그인된 상태입니다.');
    next('/');
  } else {
    next(); // 정상적으로 이동
  }
});

export default router
