<template>
    <div 
      v-if="selectedNodes && selectedNodes.length > 0" 
      class="absolute top-5 right-5 bg-white shadow-md p-4 rounded-lg border"
    >
      <h4 class="text-lg font-semibold text-gray-800 mb-3">
        선택된 노드 ({{ selectedNodes.length }})
      </h4>
      <div 
        v-for="(node, index) in selectedNodes" 
        :key="index" 
        class="p-2 border rounded-md bg-gray-100 mb-2"
      >
        <p class="font-semibold">노드 {{ index + 1 }}</p>
        <p class="text-gray-600 text-sm">X: {{ node.x }}</p>
        <p class="text-gray-600 text-sm">Y: {{ node.y }}</p>
      </div>
    </div>
  </template>
  
  <script setup>
  /**
   * props:
   *   - selectedNodes: [
   *       { x: string, y: string },
   *       ...
   *     ]
   */
  const props = defineProps({
    selectedNodes: {
      type: Array,
      default: () => []
    }
  })
  </script>