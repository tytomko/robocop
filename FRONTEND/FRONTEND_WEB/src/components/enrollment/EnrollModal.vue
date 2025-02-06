<template>
    <div v-if="showModal" class="modal">
      <div class="modal-content">
        <h2>사용자 등록</h2>
        <form @submit.prevent="submitForm">
          <div class="form-group">
            <label>이름:</label>
            <input type="text" v-model="formData.name" required />
          </div>
          <div class="form-group">
            <label>생년월일:</label>
            <input type="date" v-model="formData.birthDate" required />
          </div>
          <div class="form-group">
            <label>휴대폰 번호:</label>
            <input type="tel" v-model="formData.phone" @blur="formatPhone" required />
          </div>
          <div class="form-group">
            <label>이미지 파일:</label>
            <input type="file" @change="handleFileUpload" multiple accept="image/*" />
          </div>
          <div class="modal-actions">
            <button type="submit">등록</button>
            <button type="button" @click="closeModal">취소</button>
          </div>
        </form>
      </div>
    </div>
  </template>
  
  <script setup>
  // 부모로부터 모달 열기 상태와 formData 전달받기
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
  
  // 부모로 이벤트 전달
  const emit = defineEmits(['closeModal', 'submitForm']);
  
  // 파일 선택 이벤트 처리
  const handleFileUpload = (event) => {
    props.formData.images = Array.from(event.target.files);
  };
  
  // 휴대폰 번호 포맷팅 헬퍼
  const formatPhoneNumber = (phone) => {
    const digits = phone.replace(/\D/g, '');
    if (digits.length === 11) {
      return digits.replace(/(\d{3})(\d{4})(\d{4})/, '$1-$2-$3');
    }
    return phone;
  };
  
  // input의 blur 이벤트에서 호출
  const formatPhone = () => {
    props.formData.phone = formatPhoneNumber(props.formData.phone);
  };
  
  // 모달 닫기
  const closeModal = () => {
    emit('closeModal');
  };
  
  // 폼 제출 처리
  const submitForm = () => {
    props.formData.phone = formatPhoneNumber(props.formData.phone);
  
    let imageUrl = '';
    if (props.formData.images.length > 0) {
      const sortedImages = [...props.formData.images].sort((a, b) =>
        a.name.localeCompare(b.name)
      );
      imageUrl = URL.createObjectURL(sortedImages[0]);
    }
  
    const newUser = {
      id: Date.now(), // 간단한 고유 ID 생성
      name: props.formData.name,
      birthDate: props.formData.birthDate,
      phone: props.formData.phone,
      image: imageUrl,
    };
  
    emit('submitForm', newUser);
    emit('closeModal');
  };
  </script>
  
  <style scoped>
  /* 여기에 기존 스타일 그대로 작성 */
  .modal {
    position: fixed;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    background-color: rgba(0, 0, 0, 0.5);
    display: flex;
    align-items: center;
    justify-content: center;
    z-index: 999;
  }
  
  .modal-content {
    background: #fff;
    padding: 15px;
    border-radius: 5px;
    width: 400px;
  }
  
  .modal-content h2 {
    margin: 15px 0px;
  }
  
  .form-group {
    margin-bottom: 15px;
  }
  
  .modal-actions {
    display: flex;
    justify-content: flex-end;
  }
  
  button {
    padding: 8px 16px;
    margin: 5px;
    cursor: pointer;
  }
  </style>
  