<template>
  <div class="app-container">
    <div class="theme-toggle">
      <button @click="themeStore.toggleTheme" class="theme-button">
        {{ themeStore.isDarkMode ? '🌞' : '🌙' }}
      </button>
    </div>
    <router-view></router-view>
  </div>
</template>

<script setup>
import { onMounted } from 'vue'
import { useThemeStore } from '@/stores/theme'

const themeStore = useThemeStore()

onMounted(() => {
  themeStore.initializeTheme()
})
</script>

<style>
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

body {
  font-family: Arial, sans-serif;
  line-height: 1.6;
}

button {
  cursor: pointer;
}

input, select {
  font-size: 1rem;
}

.app-container {
  min-height: 100vh;
}

.theme-toggle {
  position: fixed;
  top: 20px;
  right: 20px;
  z-index: 1000;
}

.theme-button {
  background: none;
  border: none;
  font-size: 24px;
  cursor: pointer;
  padding: 8px;
  border-radius: 50%;
  transition: background-color 0.3s ease;
}

.theme-button:hover {
  background-color: rgba(128, 128, 128, 0.2);
}
</style>
