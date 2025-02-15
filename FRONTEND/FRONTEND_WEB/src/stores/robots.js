import { defineStore } from 'pinia'
import { ref } from 'vue'
import axios from 'axios'

export const useRobotsStore = defineStore('robots', () => {
  const registered_robots = ref([])
  const showRobotManagementModal = ref(false)
  const showNicknameModal = ref(false)
  const selectedRobotForNickname = ref(null)
  const robotNicknames = ref(JSON.parse(localStorage.getItem('robot_nicknames')) || {})
  const showModal = ref(false)
  const newRobot = ref({
    nickname: '',
    ipAddress: '',
  })

  // 🚨 selectedRobot을 우선 숫자로 초기화
  // localStorage에서 가져온 문자열을 parseInt로 변환
  const savedRobot = localStorage.getItem('selectedRobot')
  const selectedRobot = ref(savedRobot ? parseInt(savedRobot, 10) : 0) 
  // 숫자가 없으면 0(또는 null, '') 등으로 지정

  let pollingInterval = null
  const POLLING_INTERVAL = 5000 // 5초마다 폴링

  // 로봇 리스트 불러오기
  const loadRobots = async () => {
    try {
      const res = await axios.get('https://robocop-backend-app.fly.dev/api/v1/robots/')
      registered_robots.value = res.data.data.map((robot) => ({
        seq: robot.seq,
        name: robot.manufactureName,
        nickname: robot.nickname || '',
        ipAddress: robot.ipAddress || '알 수 없음',
        status: robot.status || 'waiting',
        battery: robot.battery?.level || 100,
        isCharging: robot.battery?.isCharging || false,
        lastCharged: robot.battery?.lastCharged || '알 수 없음',
        networkStatus: robot.networkStatus || 'connected',
        networkHealth: robot.networkHealth || 100,
        position: robot.position
          ? `x: ${robot.position.x}, y: ${robot.position.y}`
          : '알 수 없음',
        cpuTemp: robot.cpuTemp || 0.0,
        imageUrl: robot.image?.url || '',
        startAt: robot.startAt || '알 수 없음',
        lastActive: robot.lastActive || '알 수 없음',
        isActive: robot.IsActive || false,
        isDeleted: robot.IsDeleted || false,
        deletedAt: robot.DeletedAt || null,
        createdAt: robot.createdAt || null,
        updatedAt: robot.updatedAt || null,
        waypoints: robot.waypoints || [],
      }))
    } catch (err) {
      console.error('로봇 데이터 로드 에러:', err)
      if (err.response) {
        console.log('서버 응답:', err.response.data)
        console.log('상태 코드:', err.response.status)
      }
    }
  }

  // 로봇 닉네임 설정
  const setRobotNickname = (robotSeq, nickname) => {
    robotNicknames.value[robotSeq] = nickname
    localStorage.setItem('robot_nicknames', JSON.stringify(robotNicknames.value))
    const robot = registered_robots.value.find(r => r.seq === robotSeq)
    if (robot) robot.nickname = nickname
  }

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

    axios.post('https://robocop-backend-app.fly.dev/api/v1/robots', formData, {
      headers: { 'Content-Type': 'multipart/form-data' }
    })
      .then((response) => {
        const registeredRobot = response.data.data
        registered_robots.value.push({
          seq: registeredRobot.seq,
          name: registeredRobot.manufactureName,
          nickname: registeredRobot.nickname,   // 주의: robot -> registeredRobot
          ipAddress: registeredRobot.ipAddress,
          status: registeredRobot.status || 'waiting',
          battery: registeredRobot.battery?.level || 100,
          isCharging: registeredRobot.battery?.isCharging || false,
          networkStatus: registeredRobot.networkStatus || 'connected',
          networkHealth: registeredRobot.networkHealth || 100,
          position: registeredRobot.position
            ? `x: ${registeredRobot.position.x}, y: ${registeredRobot.position.y}`
            : '알 수 없음',
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

  const setBreakdown = (robotSeq) => {
    const robot = registered_robots.value.find(r => r.seq === robotSeq)
    if (robot) robot.status = 'error'
  }

  const setActive = (robotSeq) => {
    const robot = registered_robots.value.find(r => r.seq === robotSeq)
    if (robot) robot.status = 'navigating'
  }

  // 웹소켓으로 받은 데이터로 로봇 상태 업데이트
  const updateRobotsData = (data) => {
    if (Array.isArray(data)) {
      registered_robots.value = data.map(mapRobotData)
    }
  }

  // API 폴링 시작
  const startPolling = () => {
    loadRobots() // 초기 데이터 로드
    pollingInterval = setInterval(loadRobots, POLLING_INTERVAL)
  }

  // API 폴링 중지
  const stopPolling = () => {
    if (pollingInterval) {
      clearInterval(pollingInterval)
      pollingInterval = null
    }
  }

  // 로봇 데이터 매핑 함수
  const mapRobotData = (robot) => ({
    seq: robot.seq,
    name: robot.manufactureName,
    nickname: robot.nickname || '',
    ipAddress: robot.ipAddress || '알 수 없음',
    status: robot.status || 'waiting',
    battery: robot.battery?.level || 100,
    isCharging: robot.battery?.isCharging || false,
    lastCharged: robot.battery?.lastCharged || '알 수 없음',
    networkStatus: robot.networkStatus || 'connected',
    networkHealth: robot.networkHealth || 100,
    position: robot.position
      ? `x: ${robot.position.x}, y: ${robot.position.y}`
      : '알 수 없음',
    cpuTemp: robot.cpuTemp || 0.0,
    imageUrl: robot.image?.url || '',
    startAt: robot.startAt || '알 수 없음',
    lastActive: robot.lastActive || '알 수 없음',
    isActive: robot.IsActive || false,
    isDeleted: robot.IsDeleted || false,
    deletedAt: robot.DeletedAt || null,
    createdAt: robot.createdAt || null,
    updatedAt: robot.updatedAt || null,
    waypoints: robot.waypoints || [],
  })

  return {
    registered_robots,
    showModal,
    showRobotManagementModal,
    newRobot,
    // 숫자로 관리되는 selectedRobot
    selectedRobot,
    showNicknameModal,
    selectedRobotForNickname,
    robotNicknames,

    // methods
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
    setActive,
    updateRobotsData,
    startPolling,
    stopPolling,
  }
})
