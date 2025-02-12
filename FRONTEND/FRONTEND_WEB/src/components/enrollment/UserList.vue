<template>
  <div class="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 gap-5 mb-5">
    <!-- 사용자가 없으면 등록 카드 표시 -->
    <div
      v-if="users.length === 0"
      @click="openModal"
      class="flex flex-col items-center justify-center border-2 border-dashed border-blue-500 rounded-lg h-80 cursor-pointer hover:bg-blue-100"
    >
      <div class="text-5xl text-blue-500">+</div>
      <div class="mt-3 text-lg font-semibold text-blue-500">등록하기</div>
    </div>
    <!-- 사용자 목록 카드 -->
    <div
      v-for="user in users"
      :key="user.id"
      class="border rounded-lg shadow-sm p-4 text-center"
    >
      <!-- 이미지가 있으면 첫 번째 이미지의 URL을 표시 -->
      <img
        v-if="user.images && user.images.length > 0"
        :src="user.images[0].url"
        alt="User Image"
        class="w-full h-72 object-cover rounded-lg"
      />
      <div
        v-else
        class="w-full h-72 bg-gray-200 flex items-center justify-center text-gray-500"
      >
        No Image
      </div>
      <div class="mt-3 text-lg font-bold">{{ user.name }}</div>
      <div class="mt-2 text-sm text-gray-700">
        <p><strong>직급:</strong> {{ user.position }}</p>
        <p>
          <strong>휴대폰 번호:</strong>
          {{ user.phone ? user.phone : "No Phone" }}
        </p>
      </div>
    </div>
  </div>
</template>

<script setup>
defineProps({
  users: Array,
  openModal: Function,
});
</script>
