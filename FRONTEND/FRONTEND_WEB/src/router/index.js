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
      component: MonitoringView
    },
    {
      path: '/login',
      name: 'login', 
      component: LoginView
    },
    {
      path: '/management',
      name: 'management', 
      component: ManagementView
    },
    {
      path: '/:seq',
      name: 'detail', 
      component: RobotDetailView
    },
    {
      path: '/camera',
      name: 'camera', 
      component: CameraView
    },
    {
      path: '/enrollment',
      name: 'enrollment', 
      component: EnrollmentView
    },
    {
      path: '/control',
      name: 'control', 
      component: RobotControlView
    },
    {
      path: '/statistics',
      name: 'statistics', 
      component: StatisticsView
    }
  ]
})

export default router