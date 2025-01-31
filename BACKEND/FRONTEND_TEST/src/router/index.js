import { createRouter, createWebHistory } from 'vue-router'
import LoginView from '@/views/LoginView.vue'
import DashboardView from '@/views/DashboardView.vue'
import RealTimeMonitoring from '@/components/dashboard/monitoring/RealTimeMonitoring.vue'
import PointCloudViewer from '@/components/dashboard/monitoring/PointCloudViewer.vue'
import Timeline from '@/components/dashboard/monitoring/Timeline.vue'
import RobotManagementView from '@/views/robot/RobotManagementView.vue'
import RobotControlView from '@/views/robot/RobotControlView.vue'
import CourseManagement from '@/components/dashboard/robot/CourseManagement.vue'
import SensorDataView from '@/components/dashboard/robot/SensorDataView.vue'
import StatsSummary from '@/components/dashboard/stats/StatsSummary.vue'
import VideoList from '@/components/dashboard/stats/VideoList.vue'
import Settings from '@/components/dashboard/settings/Settings.vue'
import MapView from '@/components/dashboard/monitoring/MapView.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/login',
      name: 'login', 
      component: LoginView
    },
    {
      path: '/',
      component: DashboardView,
      children: [
        {
          path: '',
          redirect: '/monitoring'
        },
        {
          path: 'monitoring',
          name: 'monitoring',
          component: RealTimeMonitoring
        },
        {
          path: 'point-cloud',
          name: 'point-cloud',
          component: PointCloudViewer
        },
        {
          path: 'timeline',
          name: 'timeline',
          component: Timeline
        },
        {
          path: 'map',
          name: 'map',
          component: MapView
        },
        {
          path: 'robot-management',
          name: 'robot-management',
          component: RobotManagementView
        },
        {
          path: 'robot-control',
          name: 'robot-control',
          component: RobotControlView
        },
        {
          path: 'course-management',
          name: 'course-management',
          component: CourseManagement
        },
        {
          path: 'sensor-data',
          name: 'sensor-data',
          component: SensorDataView
        },
        {
          path: 'stats-summary',
          name: 'stats-summary',
          component: StatsSummary
        },
        {
          path: 'stats-video',
          name: 'stats-video',
          component: VideoList
        },
        {
          path: 'settings',
          name: 'settings',
          component: Settings
        }
      ]
    }
  ]
})

// 로그인 상태 체크 네비게이션 가드 추가
router.beforeEach((to, from, next) => {
  const isAuthenticated = localStorage.getItem('isAuthenticated')
  
  if (to.name !== 'login' && !isAuthenticated) {
    next({ name: 'login' })
  } else if (to.name === 'login' && isAuthenticated) {
    next({ name: 'monitoring' })
  } else {
    next()
  }
})

export default router
