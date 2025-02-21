<template>
  <div v-if="show" class="fixed inset-0 flex items-center justify-center bg-black bg-opacity-50 z-50">
    <div class="bg-white p-6 rounded-lg w-96 shadow-lg">
      <h3 class="text-lg font-semibold mb-4">로봇 등록하기</h3>
      <label class="block text-sm font-medium">로봇명:</label>
      <input v-model="newRobot.nickname" placeholder="로봇 이름 입력" 
             class="w-full p-2 border border-gray-300 rounded mt-1 focus:outline-none focus:ring-2 focus:ring-blue-500" />
      
      <label class="block text-sm font-medium mt-4">IP주소:</label>
      <input v-model="newRobot.ipAddress" placeholder="로봇 IP 주소 입력" 
             class="w-full p-2 border border-gray-300 rounded mt-1 focus:outline-none focus:ring-2 focus:ring-blue-500" />
      
      <div class="flex justify-end space-x-2 mt-4">
        <button @click="$emit('handleAddRobot')" 
                class="px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700">
          등록
        </button>
        <button @click="$emit('close')" 
                class="px-4 py-2 bg-red-600 text-white rounded hover:bg-red-700">
          닫기
        </button>
      </div>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  show: Boolean,
  newRobot: Object
});

const emit = defineEmits(['close', 'handleAddRobot']);
</script>
