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
    nickname: '',
    ipAddress: '',
  })
  const selectedRobot = ref(localStorage.getItem('selectedRobot') || '')

  // 로봇 리스트 불러오기
  const loadRobots = () => {
    axios.get('https://robocop-backend-app.fly.dev/api/v1/robots/')
      .then((res) => {
        registered_robots.value = res.data.data.map((robot) => ({
          seq: robot.seq,  // 기존 robotId에서 seq로 변경
          name: robot.manufactureName, // 제조사 이름
          nickname: robot.nickname || '',
          ipAddress: robot.ipAddress || '알 수 없음',
          status: robot.status || 'waiting', // 기본값 'waiting'
          battery: robot.battery?.level || 100,
          isCharging: robot.battery?.isCharging || false, // 충전 여부
          lastCharged: robot.battery?.lastCharged || '알 수 없음',
          networkStatus: robot.networkStatus || 'connected', // 네트워크 상태
          networkHealth: robot.networkHealth || 100, // 네트워크 건강도
          position: robot.position ? `x: ${robot.position.x}, y: ${robot.position.y}` : '알 수 없음', // 위치 정보
          cpuTemp: robot.cpuTemp || 0.0, // CPU 온도
          imageUrl: robot.image?.url || '', // 로봇 이미지 URL
          startAt: robot.startAt || '알 수 없음', // 로봇 가동 시작 시간
          lastActive: robot.lastActive || '알 수 없음', // 마지막 활성화 시간
          isActive: robot.IsActive || false,
          isDeleted: robot.IsDeleted || false,
          deletedAt: robot.DeletedAt || null,
          createdAt: robot.createdAt || null,
          updatedAt: robot.updatedAt || null,
          waypoints: robot.waypoints || [], // 이동 경로 정보
        }));
      })
      .catch((err) => {
        console.error('로봇 데이터 로드 에러:', err);
        if (err.response) {
          console.log('서버 응답:', err.response.data);
          console.log('상태 코드:', err.response.status);
        }
      });
  };

  // 로봇 닉네임 설정
  const setRobotNickname = (robotSeq, nickname) => {
    robotNicknames.value[robotSeq] = nickname;
    localStorage.setItem('robot_nicknames', JSON.stringify(robotNicknames.value));
    const robot = registered_robots.value.find(r => r.seq === robotSeq);
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

  const handleAddRobot = () => {
    const formData = new FormData()
    formData.append('nickname', newRobot.value.nickname)
    formData.append('ipAddress', newRobot.value.ipAddress)

    axios.post('https://robocop-backend-app.fly.dev/api/v1/robots', formData, {
      headers: { 'Content-Type': 'multipart/form-data' }
    })
      .then((response) => {
        const registeredRobot = response.data.data
        registered_robots.value.push({
          seq: registeredRobot.seq,
          name: registeredRobot.manufactureName,
          nickname: robot.nickname,
          ipAddress: registeredRobot.ipAddress,
          status: registeredRobot.status || 'waiting',
          battery: registeredRobot.battery?.level || 100,
          isCharging: registeredRobot.battery?.isCharging || false,
          networkStatus: registeredRobot.networkStatus || 'connected',
          networkHealth: registeredRobot.networkHealth || 100,
          position: registeredRobot.position ? `x: ${registeredRobot.position.x}, y: ${registeredRobot.position.y}` : '알 수 없음',
          cpuTemp: registeredRobot.cpuTemp || 0.0,
          imageUrl: registeredRobot.image?.url || '',
          startAt: registeredRobot.startAt || '알 수 없음',
          lastActive: registeredRobot.lastActive || '알 수 없음',
          isActive: registeredRobot.IsActive || false,
          isDeleted: registeredRobot.IsDeleted || false,
          deletedAt: registeredRobot.DeletedAt || null,
          createdAt: registeredRobot.createdAt || null,
          updatedAt: registeredRobot.updatedAt || null,
          waypoints: registeredRobot.waypoints || [],
        });

        closeModal()
        alert('로봇 등록 성공')
      })
      .catch((err) => {
        console.error('로봇 등록 실패:', err)
        alert('로봇 등록에 실패했습니다.')
      });
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
      nickname: '',
      ipAddress: ''
    }
  }

  const setBreakdown = (robotSeq) => {
    const robot = registered_robots.value.find(r => r.seq === robotSeq)
    if (robot) robot.status = 'error'
  }

  const setActive = (robotSeq) => {
    const robot = registered_robots.value.find(r => r.seq === robotSeq)
    if (robot) robot.status = 'navigating'
  }

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
    handleAddRobot,
    setBreakdown,
    setActive
  }
})
