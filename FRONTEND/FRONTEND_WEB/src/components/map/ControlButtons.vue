<template>
  <div class="flex space-x-3 mb-1">
    <!-- 이동/순찰 버튼 -->
    <button
      class="control-btn"
      :class="{
        'bg-gray-300 text-gray-500 cursor-not-allowed': selectedNodes.length === 0,
        'bg-blue-500': selectedNodes.length === 1,
        'bg-green-500': selectedNodes.length >= 2
      }"
      :disabled="selectedNodes.length === 0"
      @click="selectedNodes.length === 1 ? handleNavigate() : handlePatrol()"
    >
      <i class="mdi" :class="selectedNodes.length >= 2 ? 'mdi-routes' : 'mdi-navigation'"></i>
      {{ selectedNodes.length >= 2 ? '순찰' : '이동' }}
    </button>

    <!-- 리셋 버튼 -->
    <button 
      class="control-btn bg-red-500"
      :disabled="selectedNodes.length === 0"
      :class="{ 'bg-gray-300 text-gray-500 cursor-not-allowed': selectedNodes.length === 0 }"
      @click="$emit('reset')"
    >
      <i class="mdi mdi-refresh"></i>
      리셋
    </button>

    <!-- 일시정지/재가동 토글 버튼 -->
    <button 
      class="control-btn"
      :class="isPaused ? 'bg-green-500' : 'bg-orange-500'"
      @click="handlePauseToggle"
    >
      <i class="mdi" :class="isPaused ? 'mdi-play-circle' : 'mdi-pause-circle'"></i>
      {{ isPaused ? '재가동' : '일시정지' }}
    </button>
  </div>
</template>

<script setup>
import { ref } from 'vue'

const props = defineProps({
  selectedNodes: {
    type: Array,
    default: () => []
  }
})

const emits = defineEmits(['navigate', 'patrol', 'reset', 'tempStop', 'resume'])

const isPaused = ref(false)

function handleNavigate() {
  emits('navigate')
}

function handlePatrol() {
  emits('patrol')
}

function handlePauseToggle() {
  if (isPaused.value) {
    emits('resume')
  } else {
    emits('tempStop')
  }
  isPaused.value = !isPaused.value
}
</script>

<style scoped>
/* 기본 버튼 스타일 */
.control-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  width: 100px;
  height: 45px;
  font-size: 16px;
  font-weight: bold;
  color: white;
  border-radius: 10px;
  transition: all 0.3s ease-in-out;
  box-shadow: 0px 4px 6px rgba(0, 0, 0, 0.1);
}

/* 호버 효과 */
.control-btn:hover {
  transform: scale(1.05);
  box-shadow: 0px 6px 12px rgba(0, 0, 0, 0.2);
}

.control-btn:active {
  transform: scale(0.95);
  box-shadow: none;
}

/* 비활성화 상태 */
.control-btn:disabled {
  background-color: #d1d5db;
  color: #9ca3af;
  cursor: not-allowed;
}
</style>