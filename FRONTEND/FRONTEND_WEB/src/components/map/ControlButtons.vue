<template>
    <div class="flex space-x-3 mb-4">
      <!-- 이동 버튼 -->
      <button 
        class="control-btn"
        :class="{ 'bg-blue-600 text-white': isSingleSelected }"
        :disabled="!isSingleSelected"
        @click="$emit('navigate')"
      >
        <i class="mdi mdi-navigation text-lg"></i>
        이동
      </button>
      <!-- 순찰 버튼 -->
      <button 
        class="control-btn"
        :class="{ 'bg-green-600 text-white': isMultiSelected }"
        :disabled="!isMultiSelected"
        @click="$emit('patrol')"
      >
        <i class="mdi mdi-routes text-lg"></i>
        순찰
      </button>
      <!-- 리셋 버튼 -->
      <button 
        class="control-btn bg-red-500 text-white"
        @click="$emit('reset')"
      >
        <i class="mdi mdi-refresh text-lg"></i>
        리셋
      </button>
      <!-- 일시정지 버튼 -->
      <button 
        class="control-btn bg-orange-500 text-white"
        @click="$emit('tempStop')"
      >
        <i class="mdi mdi-pause-circle text-lg"></i>
        일시정지
      </button>
    </div>
  </template>
  
  <script setup>
  /**
   * props:
   *   - isSingleSelected: 단일 노드 선택 여부
   *   - isMultiSelected: 2개 이상 노드 선택 여부
   * emits:
   *   - navigate
   *   - patrol
   *   - reset
   *   - tempStop
   */
  const props = defineProps({
    isSingleSelected: {
      type: Boolean,
      default: false
    },
    isMultiSelected: {
      type: Boolean,
      default: false
    }
  })
  
  const emits = defineEmits(['navigate','patrol','reset','tempStop'])
  </script>