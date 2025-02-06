<template>
    <div class="table-section">
      <table>
        <thead>
          <tr>
            <th>선택</th>
            <th>로봇 ID</th>
            <th>작동 시간</th>
            <th>프론트캠</th>
            <th>리어캠</th>
            <th>로그</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="record in filteredRecords" :key="record.id">
            <td><input type="checkbox" /></td>
            <td>{{ record.name }}</td>
            <td>{{ record.startTime }}</td>
            <td><button>다운로드</button></td>
            <td><button>다운로드</button></td>
            <td><button>다운로드</button></td>
          </tr>
        </tbody>
      </table>
    </div>
  </template>
  
  <script setup>
  import { computed } from 'vue'
  
  const props = defineProps({
    robots: Array,
    selectedRobot: String
  })
  
  // 선택한 로봇에 따라 필터링된 테이블 데이터 반환
  const filteredRecords = computed(() => {
    return props.selectedRobot === 'all' ? props.robots : props.robots.filter(robot => robot.id === props.selectedRobot)
  })
  </script>
  
  <style scoped>
  .table-section {
    margin-top: 20px;
  }
  
  .table-section table {
    width: 100%;
    border-collapse: collapse;
    background: #fff;
    border-radius: 8px;
    overflow: hidden;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  }
  
  .table-section th {
    background-color: #f4f4f4;
    padding: 10px;
    border-bottom: 2px solid #ddd;
    text-align: center;
    font-size: 14px;
  }
  
  .table-section td {
    padding: 8px;
    border-bottom: 1px solid #ddd;
    text-align: center;
    font-size: 14px;
  }
  </style>
  