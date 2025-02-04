import { createRouter, createWebHistory } from 'vue-router'
import LoginView from '@/views/LoginView.vue'
import MonitoringView from '@/views/MonitoringView.vue'
import SettingsView from '@/views/SettingsView.vue'
import RobotDetailView from '@/views/RobotDetailView.vue'
import CameraView from '@/views/CameraView.vue'
import EnrollmentView from '@/views/EnrollmentView.vue'

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
      path: '/settings',
      name: 'settings', 
      component: SettingsView
    },
    {
      path: '/:robotId',
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
    }
  ]
})

export default router