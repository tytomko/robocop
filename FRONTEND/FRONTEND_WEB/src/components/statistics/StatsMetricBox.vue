<template>
  <div class="w-48 h-20 p-4 bg-white rounded-lg shadow-md border border-gray-200 flex flex-col justify-center text-left">
    <h3 class="text-sm text-gray-600 m-0">{{ title }}</h3>
    <p class="text-lg font-bold text-gray-800 mt-1">{{ count }}대</p>
  </div>
</template>

<script setup>
defineProps({
  title: String,
  count: Number
})
</script>
