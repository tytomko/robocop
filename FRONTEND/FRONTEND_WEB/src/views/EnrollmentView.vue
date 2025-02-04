<template>
<div class="user-registration">
    <h1>등록된 사용자 명단</h1>
    <!-- 사용자 목록 표 -->
    <table>
    <thead>
        <tr>
        <th>이름</th>
        <th>생년월일</th>
        <th>휴대폰 번호</th>
        </tr>
    </thead>
    <tbody>
        <tr v-for="user in users" :key="user.id">
        <td>{{ user.name }}</td>
        <td>{{ user.birthDate }}</td>
        <td>{{ user.phone }}</td>
        </tr>
    </tbody>
    </table>
    <!-- 등록하기 버튼 -->
    <button @click="openModal">등록하기</button>

    <!-- 모달 창 -->
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
            <input type="tel" v-model="formData.phone" required />
        </div>
        <div class="form-group">
            <label>이미지 파일:</label>
            <input type="file" @change="handleFileUpload" multiple />
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

// 임의의 사용자 데이터 (API 연동 전까지 임시 데이터)
const users = ref([
{ id: 1, name: '홍길동', birthDate: '1990-01-01', phone: '010-1234-5678' },
{ id: 2, name: '김철수', birthDate: '1985-05-10', phone: '010-2345-6789' },
]);

const showModal = ref(false);

// 모달 창에서 사용할 폼 데이터 (이미지 파일은 여러 개 첨부 가능)
const formData = reactive({
name: '',
birthDate: '',
phone: '',
images: [],
});

// 모달 열기 (arrow function과 console.log 추가)
const openModal = () => {
console.log('openModal 호출됨');
showModal.value = true;
};

// 모달 닫기 및 폼 데이터 리셋
const closeModal = () => {
showModal.value = false;
resetForm();
};

// 폼 데이터 초기화 함수
const resetForm = () => {
formData.name = '';
formData.birthDate = '';
formData.phone = '';
formData.images = [];
};

// 파일 선택 이벤트 처리 (여러 파일 첨부)
const handleFileUpload = (event) => {
formData.images = Array.from(event.target.files);
};

// 폼 제출 시 처리 (API POST 요청 자리)
const submitForm = () => {
console.log('POST 요청 보낼 데이터:', { ...formData });

// 임시로 사용자 목록에 새 사용자 추가
const newUser = {
    id: Date.now(), // 간단한 고유 ID 생성
    name: formData.name,
    birthDate: formData.birthDate,
    phone: formData.phone,
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

/* 아래 robot 테이블 CSS와 동일하게 적용 */
table {
width: 100%;
border-collapse: collapse;
margin-bottom: 20px;
}

h2 {
    margin: 10px 0px;
}

table th,
table td {
border: 1px solid #ddd;
padding: 10px;
text-align: center;
}

table th {
background: #f4f4f4;
}

button {
padding: 8px 16px;
margin: 5px;
cursor: pointer;
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
z-index: 2;
}

.modal-content {
background: #fff;
padding: 15px;
border-radius: 5px;
width: 400px;
}

.form-group {
margin-bottom: 15px;
}

.modal-actions {
display: flex;
justify-content: flex-end;
}
</style>
