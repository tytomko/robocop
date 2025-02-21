<template>
  <div 
    v-if="selectedNodes && selectedNodes.length > 0" 
    class="fixed top-[75px] right-5 max-w-[250px] bg-white/90 backdrop-blur-sm shadow-lg p-4 rounded-lg border border-gray-200 transition-all duration-300 hover:bg-white"
  >
    <div class="flex justify-between items-center mb-3">
      <h4 class="text-sm font-semibold text-gray-800">
        선택된 노드 ({{ selectedNodes.length }})
      </h4>
      <div class="h-5 w-5 bg-blue-500 rounded-full flex items-center justify-center text-white text-xs">
        {{ selectedNodes.length }}
      </div>
    </div>
    
    <div class="max-h-[calc(100vh-160px)] overflow-y-auto pr-2 space-y-2">
      <div 
        v-for="(node, index) in selectedNodes" 
        :key="index" 
        class="group p-2 border border-gray-200 rounded-md bg-gray-50 hover:bg-gray-100 transition-colors duration-200 relative"
      >
        <div class="flex items-center pr-6">
          <span class="w-6 h-6 flex items-center justify-center bg-blue-100 text-blue-600 rounded-full text-xs font-medium mr-2">
            {{ index + 1 }}
          </span>
          <div class="flex-1 text-xs">
            <p class="text-gray-600">X: {{ node.x }}</p>
            <p class="text-gray-600">Y: {{ node.y }}</p>
          </div>
          <button 
            @click="removeNode(node, index)"
            class="absolute right-2 top-1/2 -translate-y-1/2 opacity-0 group-hover:opacity-100 transition-opacity duration-200 p-1 hover:bg-gray-200 rounded-full"
          >
            <svg xmlns="http://www.w3.org/2000/svg" class="h-4 w-4 text-gray-500 hover:text-red-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  selectedNodes: {
    type: Array,
    default: () => []
  }
})

const emit = defineEmits(['remove-node'])

const removeNode = (node, index) => {
  emit('remove-node', { node, index })
}
</script>