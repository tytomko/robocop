import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useThemeStore = defineStore('theme', () => {
  const isDarkMode = ref(localStorage.getItem('darkMode') === 'true')

  const toggleTheme = () => {
    isDarkMode.value = !isDarkMode.value
    localStorage.setItem('darkMode', isDarkMode.value)
    document.documentElement.classList.toggle('dark-mode', isDarkMode.value)
  }

  // 초기 테마 설정
  const initializeTheme = () => {
    document.documentElement.classList.toggle('dark-mode', isDarkMode.value)
  }

  return {
    isDarkMode,
    toggleTheme,
    initializeTheme
  }
}) 