<template>
  <div class="fixed inset-0 bg-black bg-opacity-50 flex justify-center items-center z-50">
    <div class="bg-white p-6 rounded-lg w-96 shadow-lg">
      <div class="flex justify-between items-center mb-4">
        <h3 class="text-lg font-semibold">로봇 설정</h3>
      </div>
      <label class="block text-sm font-medium text-gray-700">로봇명:</label>
      <input 
        v-model="robot.nickname" 
        placeholder="로봇명을 설정하세요" 
        class="w-full mt-2 p-2 border border-gray-300 rounded-md focus:ring focus:ring-blue-300 focus:outline-none"
      />
      <div class="flex justify-end gap-3 mt-4">
        <button @click="saveNickname" class="px-4 py-2 bg-blue-500 text-white rounded-md hover:bg-blue-600">저장</button>
        <button @click="closeModal" class="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-600">닫기</button>
      </div>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  show: Boolean,
  robot: Object
});

const emit = defineEmits(['save', 'close']);

const saveNickname = () => {
  emit('save', props.robot.id, props.robot.nickname);
};

const closeModal = () => {
  emit('close');
};
</script>
