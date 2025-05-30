<template>
  <div
    v-if="showModal"
    class="fixed inset-0 flex items-center justify-center bg-black bg-opacity-50 z-50"
  >
    <div class="bg-white p-6 rounded-lg shadow-lg w-96">
      <h2 class="text-xl font-semibold mb-4">사용자 등록</h2>
      <form @submit.prevent="submitForm" class="space-y-4">
        <div>
          <label class="block font-medium mb-1">이름:</label>
          <input
            type="text"
            v-model="formData.personName"
            required
            class="w-full px-3 py-2 border rounded-lg focus:ring focus:ring-blue-300"
          />
        </div>
        <div>
          <label class="block font-medium mb-1">직급:</label>
          <input
            type="text"
            v-model="formData.personPosition"
            required
            class="w-full px-3 py-2 border rounded-lg focus:ring focus:ring-blue-300"
          />
        </div>
        <div>
          <label class="block font-medium mb-1">휴대폰 번호:</label>
          <input
            type="tel"
            v-model="formData.personPhone"
            @blur="formatPhone"
            required
            class="w-full px-3 py-2 border rounded-lg focus:ring focus:ring-blue-300"
          />
        </div>
        <div>
          <label class="block font-medium mb-1">등록자 사진:</label>
          <!-- multiple 속성으로 여러 파일 선택 지원 -->
          <input
            type="file"
            @change="handleFileUpload"
            multiple
            accept="image/*"
            required
            class="w-full border p-2 rounded-lg cursor-pointer"
          />
        </div>
        <div class="flex justify-end space-x-2">
          <button
            type="submit"
            class="px-4 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600"
          >
            등록
          </button>
          <button
            type="button"
            @click="closeModal"
            class="px-4 py-2 bg-gray-400 text-white rounded-lg hover:bg-gray-500"
          >
            취소
          </button>
        </div>
      </form>
    </div>
  </div>
</template>

<script setup>
const props = defineProps({
  showModal: {
    type: Boolean,
    required: true,
  },
  formData: {
    type: Object,
    required: true,
  },
});

const emit = defineEmits(["closeModal", "submitForm"]);

const handleFileUpload = (event) => {
  props.formData.images = Array.from(event.target.files);
};

const formatPhoneNumber = (phone) => {
  const digits = phone.replace(/\D/g, "");
  if (digits.length === 11) {
    return digits.replace(/(\d{3})(\d{4})(\d{4})/, "$1-$2-$3");
  }
  return phone;
};

const formatPhone = () => {
  props.formData.personPhone = formatPhoneNumber(props.formData.personPhone);
};

const closeModal = () => {
  emit("closeModal");
};

const submitForm = () => {
  // 휴대폰 번호 포맷 적용
  props.formData.personPhone = formatPhoneNumber(props.formData.personPhone);

  const newUser = {
    personName: props.formData.personName,
    personPosition: props.formData.personPosition,
    personPhone: props.formData.personPhone,
    images: props.formData.images, // 여러 파일 객체 전달
    personDepartment: "",
  };

  emit("submitForm", newUser);
  emit("closeModal");
};
</script>
