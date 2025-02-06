<template>
<div class="user-registration">
    <!-- 헤더 영역: 제목과 등록하기 버튼 -->
    <div class="header">
    <h1>등록된 사용자 명단</h1>
    <!-- 사용자가 있을 때만 오른쪽에 등록하기 버튼 노출 -->
    <button v-if="users.length > 0" @click="openModal" class="header-button">등록하기</button>
    </div>

    <!-- 카드 그리드 -->
    <div class="card-grid">
    <!-- 사용자 목록이 비어있을 때: 빈 카드에 + 등록하기 표시 -->
    <div v-if="users.length === 0" class="user-card add-user-card" @click="openModal">
        <div class="user-add-icon">+</div>
        <div class="user-add-text">등록하기</div>
    </div>
    <!-- 사용자 목록이 있을 경우: 각 사용자 카드를 직접 렌더링 -->
    <div v-for="user in users" :key="user.id" class="user-card">
        <img v-if="user.image" :src="user.image" alt="User Image" class="user-image" />
        <div v-else class="user-placeholder">No Image</div>
        <!-- 이름을 크게 bold체로 표시 -->
        <div class="user-name">{{ user.name }}</div>
        <!-- 생년월일과 휴대폰 번호 출력 (라벨 포함) -->
        <div class="user-details">
        <div class="user-birthdate"><strong>생년월일: </strong>{{ user.birthDate }}</div>
        <div class="user-phone"><strong>휴대폰 번호: </strong>{{ user.phone }}</div>
        </div>
    </div>
    </div>

    <!-- 등록 모달 -->
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
            <!-- 휴대폰 번호 입력 후 블러 이벤트로 자동 포맷 -->
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
</div>
</template>

<script setup>
import { ref, reactive } from 'vue';

// 사용자 목록 (초기에는 빈 배열)
const users = ref([]);

// 모달 표시 여부
const showModal = ref(false);

// 폼 데이터 (이미지 파일은 여러 개 첨부 가능)
const formData = reactive({
name: '',
birthDate: '',
phone: '',
images: [],
});

// 모달 열기
const openModal = () => {
showModal.value = true;
};

// 폼 데이터 리셋
const resetForm = () => {
formData.name = '';
formData.birthDate = '';
formData.phone = '';
formData.images = [];
};

// 모달 닫기
const closeModal = () => {
showModal.value = false;
resetForm();
};

// 파일 선택 이벤트 처리
const handleFileUpload = (event) => {
formData.images = Array.from(event.target.files);
};

// 휴대폰 번호 포맷팅 헬퍼: 숫자만 남기고 11자리일 경우 010-1111-2222 형식으로 변환
const formatPhoneNumber = (phone) => {
const digits = phone.replace(/\D/g, '');
if (digits.length === 11) {
    return digits.replace(/(\d{3})(\d{4})(\d{4})/, '$1-$2-$3');
}
return phone;
};

// input의 blur 이벤트에서 호출
const formatPhone = () => {
formData.phone = formatPhoneNumber(formData.phone);
};

// 폼 제출 처리: 첨부된 이미지 중 파일명이 가장 빠른 이미지를 선택
const submitForm = () => {
// 휴대폰 번호 포맷팅 적용 (혹시 blur가 발생하지 않았을 경우 대비)
formData.phone = formatPhoneNumber(formData.phone);

let imageUrl = '';
if (formData.images.length > 0) {
    const sortedImages = [...formData.images].sort((a, b) =>
    a.name.localeCompare(b.name)
    );
    imageUrl = URL.createObjectURL(sortedImages[0]);
}
const newUser = {
    id: Date.now(), // 간단한 고유 ID 생성
    name: formData.name,
    birthDate: formData.birthDate,
    phone: formData.phone,
    image: imageUrl,
};
users.value.push(newUser);
closeModal();
};
</script>

<style scoped>
.user-registration {
padding: 20px;
font-family: sans-serif;
}

/* 헤더 영역: 제목과 등록하기 버튼을 같은 행에 배치 */
.header {
display: flex;
align-items: center;
justify-content: space-between;
margin-bottom: 20px;
}

.header h1 {
margin: 0;
}

.header-button {
padding: 8px 16px;
cursor: pointer;
}

/* 카드 그리드: 한 줄에 최대 3개 카드 */
.card-grid {
display: grid;
grid-template-columns: repeat(3, 1fr);
gap: 20px;
margin-bottom: 20px;
}

/* 사용자 카드 기본 스타일 */
.user-card {
border: 1px solid #ddd;
border-radius: 8px;
overflow: hidden;
text-align: center;
padding: 10px;
box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

/* 사용자 이미지 */
.user-image {
width: 100%;
height: 300px;
object-fit: cover;
}

/* 이미지 없을 경우 플레이스홀더 */
.user-placeholder {
width: 100%;
height: 300px;
background: #f0f0f0;
display: flex;
align-items: center;
justify-content: center;
color: #999;
}

/* 사용자 이름 */
.user-name {
margin-top: 15px;
font-size: 16px;
font-weight: bold;
}

/* 사용자 상세 정보 */
.user-details {
margin-top: 8px;
font-size: 14px;
color: #333;
}

/* 등록하기 카드 (빈 카드) */
.add-user-card {
width: 93%;
height: 350px;
display: flex;
flex-direction: column;
align-items: center;
justify-content: center;
cursor: pointer;
}

.user-add-icon {
font-size: 48px;
color: #007BFF;
}

.user-add-text {
margin-top: 10px;
font-size: 18px;
color: #007BFF;
font-weight: bold;
}

/* 모달 스타일 */
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
