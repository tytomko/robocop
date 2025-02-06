<template>
    <div class="metric-box">
      <h3>{{ title }}</h3>
      <p>{{ count }}대</p>
    </div>
  </template>
  
  <script setup>
  defineProps({
    title: String,
    count: Number
  })
  </script>
  
  <style scoped>
  .metric-box {
    width: 200px;
    height: 80px;
    padding: 10px 15px;
    background: #fff;
    border-radius: 8px;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
    text-align: left;
    display: flex;
    flex-direction: column;
    justify-content: center;
    border: 1px solid #e0e0e0;
  }
  
  /* Metric Box Title */
  .metric-box h3 {
    font-size: 14px;
    color: #555;
    margin: 0;
  }
  
  /* Metric Box Number */
  .metric-box p {
    font-size: 18px;
    font-weight: bold;
    color: #333;
    margin: 5px 0 0;
  }
  </style>
  