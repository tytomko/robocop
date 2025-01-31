import { createRouter, createWebHistory } from 'vue-router'
import LoginView from '@/views/LoginView.vue'
import MonitoringView from '@/views/MonitoringView.vue'
import SettingsView from '@/views/SettingsView.vue'
import RobotDetailView from '@/views/RobotDetailView.vue'

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
    }
  ]
})

export default router