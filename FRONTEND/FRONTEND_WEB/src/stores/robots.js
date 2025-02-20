import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import axios from 'axios'

export const useRobotsStore = defineStore('robots', () => {
  const registered_robots = ref([])  // DB에서 가져온 로봇 목록
  const websocket_robots = ref([])   // 웹소켓으로 받은 로봇 정보
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
  const webSocketConnected = ref(false)
  let pollingInterval = null
  const POLLING_INTERVAL = 5000
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
      registered_robots.value = res.data.data.map((robot) => ({
        seq: robot.seq,
        manufactureName: robot.manufactureName,
        nickname: robot.nickname || '',
        sensorName: robot.sensorName || '',
        ipAddress: robot.ipAddress || '알 수 없음',
        networkStatus: robot.networkStatus || 'disconnected',
        status: robot.status || 'waiting',
        battery: robot.battery?.level || 100,
        isCharging: robot.battery?.isCharging || false,
        networkHealth: robot.networkHealth || 100,
        position: robot.position
          ? `x: ${robot.position.x}, y: ${robot.position.y}`
          : '알 수 없음',
        orientation: robot.position?.orientation || '알 수 없음',
        motion: robot.motion
          ? `kph: ${robot.motion.kph}, mps : ${robot.motion.mps}`
          : '알 수 없음',
        cpuTemp: robot.cpuTemp || 0.0,
        waypoints : robot.waypoints || [], 
        startAt: robot.startAt || '알 수 없음',
        isActive: robot.IsActive || false
      }))
    } catch (err) {
      console.error('로봇 데이터 로드 에러:', err)
    }
  }

  // 웹소켓 데이터와 DB 데이터 병합
  const displayRobots = computed(() => {
    if (webSocketConnected.value) {
      return registered_robots.value.map(robot => {
        // DB의 manufactureName과 웹소켓의 manufactureName 매칭
        const wsRobot = websocket_robots.value.find(wr => 
          wr.manufactureName === robot.manufactureName
        )
        
        if (wsRobot) {
          return {
            ...robot,
            isWebSocketConnected: true,                // 웹소켓 연결 상태
            battery: wsRobot.battery,                  // 배터리 상태 (0-100%)
            networkHealth: wsRobot.network,            // 네트워크 상태 (0-100%)
            status: wsRobot.status,                    // 로봇 상태 (charging, moving 등)
            position: wsRobot.position || {            // UTM 좌표 (x,y)
              x: wsRobot.rawPosition?.x,
              y: wsRobot.rawPosition?.y
            },
            cpuTemp: wsRobot.temperature,              // CPU 온도
            isActive: wsRobot.is_active,               // 로봇 활성화 상태
            rawPosition: wsRobot.rawPosition,          // 원본 UTM 좌표 (x,y,z)
            orientation: wsRobot.orientation            // 로봇 방향 정보
          }
        }
        return { ...robot, isWebSocketConnected: false }
      })
    }
    return registered_robots.value
  })

  // API 폴링 관련
  const startPolling = () => {
    loadRobots()
    pollingInterval = setInterval(loadRobots, POLLING_INTERVAL)
  }

  const stopPolling = () => {
    if (pollingInterval) {
      clearInterval(pollingInterval)
      pollingInterval = null
    }
  }  

  // 웹소켓 데이터 업데이트
  const updateRobotWebSocketData = (robotId, data, topicType) => {
    let normalizedData = {}

    switch (topicType) {
      case 'status':
        normalizedData = {
          manufactureName: robotId,
          seq: data.robot_id,
          battery: Number(data.battery || 0),
          network: Number(data.network || 0),
          temperature: Number(data.temperature || 0),
          is_active: Boolean(data.is_active),
          status: data.status || 'unknown'
        }

        const statusIndex = websocket_robots.value.findIndex(r => 
          r.manufactureName === robotId && r.seq === data.robot_id
        )
        
        if (statusIndex >= 0) {
          websocket_robots.value[statusIndex] = {
            ...websocket_robots.value[statusIndex],
            ...normalizedData
          }
        } else {
          websocket_robots.value.push(normalizedData)
        }
        break

      case 'utm_pose':
        const utmIndex = websocket_robots.value.findIndex(r => 
          r.manufactureName === robotId
        )
        if (utmIndex >= 0) {
          websocket_robots.value[utmIndex] = {
            ...websocket_robots.value[utmIndex],
            position: data.position,
            rawPosition: data.rawPosition,
            orientation: data.orientation
          }
        }
        break
    }
  }

  const setWebSocketConnected = (status) => {
    webSocketConnected.value = status
    if (!status) {
      websocket_robots.value = []
    }
  }

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
        registered_robots.value.push({
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

  const alerts = ref(false);

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
    websocket_robots,
    displayRobots,
    webSocketConnected,
    leftSidebarCollapsed,
    alerts,

    // methods
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
    updateRobotWebSocketData,
    setWebSocketConnected,
    startPolling,
    stopPolling,
  }
})
