import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import axios from 'axios'

export const useRobotsStore = defineStore('robots', () => {
  const robots = ref([])
  const showRobotManagementModal = ref(false)
  const showNicknameModal = ref(false)
  const selectedRobotForNickname = ref(null)
  const robotNicknames = ref(JSON.parse(localStorage.getItem('robot_nicknames')) || {})
  const showModal = ref(false)
  const newRobot = ref({
    nickname: '',
    ipAddress: '',
  })
  const savedRobot = localStorage.getItem('selectedRobot')
  const selectedRobot = ref(savedRobot ? parseInt(savedRobot, 10) : 0)  // localStorage에서 가져온 문자열을 parseInt로 변환
  let pollingInterval = null
  const POLLING_INTERVAL = 500
 // localStorage와 연동되는 사이드바 상태
  const storedLeftState = localStorage.getItem("left-sidebar-collapsed");
  const leftSidebarCollapsed = ref(storedLeftState === "true");
  // App.vue의 토글 함수와 연동
  const updateSidebarStates = (left) => {
    leftSidebarCollapsed.value = left;
  };

  // 로봇 리스트 불러오기
  const loadRobots = async () => {
    try {
      const res = await axios.get('https://robocopbackendssafy.duckdns.org/api/v1/robots/')
      if (res.data?.data && Array.isArray(res.data.data)) {
        robots.value = res.data.data.map(mapRobotData)
      } else {
        console.error('예상치 못한 데이터 구조:', res.data)
        robots.value = []
      }
    } catch (err) {
      console.error('로봇 데이터 로드 에러:', err)
      robots.value = []
    }
  }

  // 웹소켓 데이터와 DB 데이터 병합
  const displayRobots = computed(() => robots.value)

  // API 폴링 관련
  const startPolling = () => {
    if (pollingInterval) {
      stopPolling()
    }

    loadRobots()

    pollingInterval = setInterval(async () => {
      try {
        const res = await axios.get('https://robocopbackendssafy.duckdns.org/api/v1/robots/')
        if (res.data?.data && Array.isArray(res.data.data)) {
          robots.value = res.data.data.map(mapRobotData)
        } else {
          console.error('예상치 못한 데이터 구조:', res.data)
        }
      } catch (error) {
        console.error('폴링 에러:', error)
      }
    }, POLLING_INTERVAL)
  }

  const stopPolling = () => {
    if (pollingInterval) {
      clearInterval(pollingInterval)
      pollingInterval = null
    }
  }  

    // 공통 매핑 함수 추가
    const mapRobotData = (robot) => ({
      seq: robot.seq,
      manufactureName: robot.manufactureName,
      nickname: robot.nickname || '',
      sensorName: robot.sensorName || '',
      ipAddress: robot.ipAddress || '알 수 없음',
      networkStatus: robot.networkStatus || 'disconnected',
      status: robot.status || 'waiting',
      networkHealth: robot.networkHealth || 100,
      position: robot.position || { x: 0, y: 0, z: 0 },
      orientation: robot.orientation || 0,
      motion: robot.motion || { kph: 0, mps: 0 },
      battery: {
        level: robot.battery?.level || 100,
        isCharging: robot.battery?.isCharging || false,
        lastCharged: robot.battery?.lastCharged || null
      },
      cpuTemp: robot.cpuTemp || 0,
      waypoints: robot.waypoints || [],
      startAt: robot.startAt || new Date().toISOString(),
      IsActive: robot.IsActive || false,
      isDeleted: robot.IsDeleted || false,
      lastActive: robot.lastActive || new Date().toISOString(),
      createdAt: robot.createdAt || new Date().toISOString(),
      updatedAt: robot.updatedAt || new Date().toISOString()
    })

  // 로봇 닉네임 설정
  const openNicknameModal = (robot) => {
    selectedRobotForNickname.value = robot
    showNicknameModal.value = true
  }

  const closeNicknameModal = () => {
    showNicknameModal.value = false
    selectedRobotForNickname.value = null
  }

  // 로봇 선택 처리 → 로컬 스토리지 저장
  const handleRobotSelection = () => {
    if (selectedRobot.value !== 0) {
      localStorage.setItem('selectedRobot', String(selectedRobot.value))
    } else {
      localStorage.removeItem('selectedRobot')
    }
  }

  const handleAddRobot = () => {
    const formData = new FormData()
    formData.append('nickname', newRobot.value.nickname)
    formData.append('ipAddress', newRobot.value.ipAddress)

    axios.post('https://robocopbackendssafy.duckdns.org/api/v1/robots/', formData, {
      headers: { 'Content-Type': 'multipart/form-data' }
    })
      .then((response) => {
        const registeredRobot = response.data.data
        robots.value.push({
          seq: registeredRobot.seq,
          manufactureName: registeredRobot.manufactureName,
          nickname: registeredRobot.nickname,   // 주의: robot -> registeredRobot
          ipAddress: registeredRobot.ipAddress,
          sensorName: registeredRobot.sensorName || '',
          status: registeredRobot.status || 'waiting',
          battery: registeredRobot.battery?.level || 100,
          isCharging: registeredRobot.battery?.isCharging || false,
          networkStatus: registeredRobot.networkStatus || 'connected',
          networkHealth: registeredRobot.networkHealth || 100,
          position: registeredRobot.position
            ? `x: ${registeredRobot.position.x}, y: ${registeredRobot.position.y}`
            : '알 수 없음',
          orientation: registeredRobot.position?.orientation || '알 수 없음',
          motion: registeredRobot.motion 
          ? `kph: ${registeredRobot.motion.kph}, mps: ${registeredRobot.motion.mps}`
          : '알 수 없음',
          cpuTemp: registeredRobot.cpuTemp || 0.0,
          waypoints : registeredRobot.waypoints || [],
          imageUrl: registeredRobot.image?.url || '',
          startAt: registeredRobot.startAt || '알 수 없음',
          lastActive: registeredRobot.lastActive || '알 수 없음',
          isActive: registeredRobot.IsActive || false
        })

        closeModal()
        alert('로봇 등록 성공')
      })
      .catch((err) => {
        console.error('로봇 등록 실패:', err)
        alert('로봇 등록에 실패했습니다.')
      })
  }

  // 로봇 관리 모달 열기/닫기
  const openRobotManagementModal = () => { showRobotManagementModal.value = true }
  const closeRobotManagementModal = () => { showRobotManagementModal.value = false }

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

const updateRobotPosition = (seq, position) => {
  const robotIndex = robots.value.findIndex(r => r.seq === seq);
  if (robotIndex !== -1) {
    robots.value[robotIndex] = {
      ...robots.value[robotIndex],
      position: {
        x: position.x,
        y: position.y,
      }
    };
  }
}

const updateRobotStatus = (seq, statusData) => {
  const robot = robots.value.find(r => r.seq === seq);
  if (robot) {
    Object.assign(robot, statusData);
  }
}

  return {
    robots,
    showModal,
    showRobotManagementModal,
    newRobot,
    // 숫자로 관리되는 selectedRobot
    selectedRobot,
    showNicknameModal,
    selectedRobotForNickname,
    robotNicknames,
    displayRobots,
    leftSidebarCollapsed,

    // methods
    updateRobotPosition,
    updateSidebarStates,
    openNicknameModal,
    closeNicknameModal,
    openRobotManagementModal,
    closeRobotManagementModal,
    loadRobots,
    openAddRobotModal,
    handleRobotSelection,
    closeModal,
    handleAddRobot,
    startPolling,
    stopPolling,
    updateRobotStatus
  }
})
