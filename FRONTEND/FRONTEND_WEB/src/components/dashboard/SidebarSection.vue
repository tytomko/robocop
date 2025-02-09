<template>
  <div class="w-96 bg-white border-l border-gray-300 flex flex-col overflow-y-auto flex-shrink-0">
    <AlarmNotification />
    <div class="flex flex-col gap-1 bg-gray-100 flex-1">
      <div class="bg-white">
        <div class="flex items-center mb-2 p-4 border-b border-gray-200 bg-white">
          <h3 class="m-0 text-gray-800 font-normal text-base">지도</h3>
        </div>
        <div class="p-4 flex flex-col gap-5">
          <RobotMap />
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import RobotMap from '@/components/map/RobotMap.vue';
import AlarmNotification from './AlarmNotification.vue';
import { useRobotsStore } from '@/stores/robots';
import { onMounted } from 'vue';

const robotsStore = useRobotsStore();

onMounted(() => {
  robotsStore.loadRobots();
  const savedRobot = localStorage.getItem("selectedRobot");
  if (savedRobot) {
    robotsStore.selectedRobot = savedRobot;
  }
});
</script>
