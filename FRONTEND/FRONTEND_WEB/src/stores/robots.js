import { defineStore } from 'pinia'
import { ref } from 'vue'
import axios from 'axios'

export const useRobotsStore = defineStore('robots', () => {
  const registered_robots = ref([])
  const showRobotManagementModal = ref(false)
  const showNicknameModal = ref(false);
  const selectedRobotForNickname = ref(null);
  const robotNicknames = ref(JSON.parse(localStorage.getItem('robot_nicknames')) || {});
  const showModal = ref(false)
  const newRobot = ref({
    name: '',
    ipAddress: '',
    image: null
  })
  const selectedRobot = ref(localStorage.getItem('selectedRobot') || '')

  // 로봇 리스트 불러오기
  const loadRobots = function () {
    axios({
      method : 'get',
      url: 'https://robocop-backend-app.fly.dev/api/v1/robots/',
    })
    .then((res)=> {
      // 데이터를 변환하여 저장
      registered_robots.value = res.data.data.map((robot) => ({
        id: robot.robotId,
        name: robot.name,
        nickname: robotNicknames.value[robot.robotId] || '',
        ipAddress: robot.ipAddress || '알 수 없음',
        status: robot.status || 'idle',
        battery: robot.battery?.level || 100,
        isCharging: robot.battery?.isCharging || false,
        location: robot.location || '알 수 없음',
      }))
    })
    .catch ((err) => {
      console.error('로봇 데이터 로드 에러:', err)
      if (err.response) {
        console.log('서버응답:', err.response.data)
        console.log('상태코드:', err.response.status  )
      }
    })
  }
  
  // 로봇 닉네임 설정
  const setRobotNickname = (robotId, nickname) => {
    robotNicknames.value[robotId] = nickname;
    localStorage.setItem('robot_nicknames', JSON.stringify(robotNicknames.value));
    const robot = registered_robots.value.find(r => r.id === robotId);
    if (robot) robot.nickname = nickname;
  };

  const openNicknameModal = (robot) => {
    selectedRobotForNickname.value = robot;
    showNicknameModal.value = true;
  };

  const closeNicknameModal = () => {
    showNicknameModal.value = false;
    selectedRobotForNickname.value = null;
  };

  // 로봇 선택 처리
  const handleRobotSelection = function () {
    if (selectedRobot.value) {
      localStorage.setItem('selectedRobot', selectedRobot.value)
    } else {
      localStorage.removeItem('selectedRobot')
    }
  }

  const handleAddRobot = async () => {
    try {
      const formData = new FormData()
      formData.append('name', newRobot.value.name)
      formData.append('ip_address', newRobot.value.ipAddress)
      const response = await axios.post('https://robocop-backend-app.fly.dev/api/v1/robots', formData, {
        headers: { 'Content-Type': 'multipart/form-data' }
      })
      const registeredRobot = response.data.data
      registered_robots.value.push({
        id: registeredRobot.id,
        name: registeredRobot.name,
        ipAddress: registeredRobot.ip_address,
        status: 'idle',
        battery: registeredRobot.battery || 50,
        location: registeredRobot.location || '알 수 없음',
      })

      closeModal()
      alert('로봇 등록 성공')
    } catch (err) {
      console.error('로봇 등록 실패:', err)
      alert('로봇 등록에 실패했습니다.')
    }
  }

    // 로봇 관리 모달 열기
    const openRobotManagementModal = () => { 
      showRobotManagementModal.value = true 
    }
  
    // 로봇 관리 모달 닫기
    const closeRobotManagementModal = () => { 
      showRobotManagementModal.value = false 
    }
  
    // 로봇 등록 모달 열기 (로봇 관리 모달을 닫고 열기)
    const openAddRobotModal = () => {
      showRobotManagementModal.value = false
      showModal.value = true
    }
  
    // 로봇 등록 모달 닫기
    const closeModal = () => {
      showModal.value = false
      newRobot.value = {
        name: '',
        ipAddress: ''
      }
    }

    const updateRobotStatus = async (robotId, newStatus) => {
      try {
        await axios.put(`https://robocop-backend-app.fly.dev/api/v1/robots/${robotId}/status`, {
          status: newStatus
        });
        const robot = registered_robots.value.find(r => r.id === robotId);
        if (robot) {
          robot.status = newStatus;  // 로컬 상태 업데이트
        }
      } catch (err) {
        console.error('로봇 상태 업데이트 실패:', err);
        alert('로봇 상태 업데이트에 실패했습니다.');
      }
    };

  return {
    registered_robots,
    showModal,
    showRobotManagementModal,
    newRobot,
    selectedRobot,
    showNicknameModal,
    selectedRobotForNickname,
    setRobotNickname,
    openNicknameModal,
    closeNicknameModal,
    openRobotManagementModal,
    closeRobotManagementModal,
    loadRobots,
    openAddRobotModal,
    handleRobotSelection,
    closeModal,
    updateRobotStatus,
    handleAddRobot
  }
}) 