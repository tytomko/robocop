<template>
    <div class="robot-info">
      <!-- 기본 정보 -->
      <div class="status-item">
        <h3 class="info-header">기본 정보</h3>
        <p><strong>상태:</strong> {{ getStatusLabel(robot.status) }}</p>
        <p><strong>배터리:</strong> {{ robot.battery }}%</p>
        <p><strong>네트워크 상태:</strong> {{ robot.network }}</p>
        <p><strong>작동 시작 시간:</strong> {{ robot.starttime }}</p>
      </div>
      <hr>
  
      <!-- 센서 데이터 -->
      <div class="status-item">
        <h3 class="info-header">센서 데이터</h3>
        <div class="sensor-status">
          <span v-for="(status, sensor) in robot.sensors" 
                :key="sensor"
                class="sensor-badge"
                :class="status">
            {{ getSensorLabel(sensor) }}: {{ status }}
          </span>
        </div>
      </div>
      <hr>
  
      <!-- 3D 라이다 정보 -->
      <div>
        <h3 class="info-header">3D 라이다</h3>
        <div class="lidar-info">
          <p v-if="robot.lidarData">
            <strong>라이다 상태:</strong> {{ robot.lidarData.status }}<br>
            <strong>라이다 거리:</strong> {{ robot.lidarData.distance }} m
          </p>
          <p v-else>라이다 정보가 없습니다.</p>
        </div>
      </div>
    </div>
  </template>
  
  <script setup>
  defineProps({
    robot: Object
  });
  
  const getStatusLabel = (status) => {
    const labels = {
      active: '활동 중',
      charging: '충전 중',
      stopped: '정지 중',
      error: '오류 발생',
      idle: '대기 중',
      returning: '복귀 중',
      breakdown: '고장'
    };
    return labels[status] || status;
  };
  
  const getSensorLabel = (sensor) => {
    const sensorLabels = {
      temperature: '온도',
      humidity: '습도',
      proximity: '근접 센서',
      camera: '카메라',
      lidar: '라이다'
    };
    return sensorLabels[sensor] || sensor;
  };
  </script>
  
  <style scoped>
  .robot-info {
    background: #f8f9fa;
    padding: 15px;
    border-radius: 8px;
  }
  
  .sensor-status {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
  }
  
  .sensor-badge {
    padding: 4px 8px;
    border-radius: 4px;
    font-size: 12px;
  }
  
  .sensor-badge.normal {
    background: #28a745;
    color: white;
  }
  
  .sensor-badge.warning {
    background: #ffc107;
    color: #000;
  }
  
  .sensor-badge.error {
    background: #dc3545;
    color: white;
  }
  
  .lidar-info {
    margin-top: 10px;
  }
  </style>
  