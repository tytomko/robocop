import { defineStore } from 'pinia'
import axios from 'axios'

export const useRobotCommandsStore = defineStore('robotCommands', () => {
  // 목적지 이동 명령 함수
  const navigateCommand = async (selectedNodes, currentRobotSeq) => {
    if (selectedNodes.length !== 1 || !currentRobotSeq) {
      console.warn('[RobotCommandsStore] 로봇이 선택되지 않았거나 목적지가 선택되지 않았습니다.')
      return
    }
    try {
      const goal = {
        x: selectedNodes[0].id[0],
        y: selectedNodes[0].id[1],
        theta: 0.0
      }
      await axios.post(
        `https://robocopbackendssafy.duckdns.org/api/v1/${currentRobotSeq}/call-service/navigate`,
        { goal }
      )
      console.log('[RobotCommandsStore] 이동 명령 전송 완료')
    } catch (error) {
      console.error('Navigation request failed:', error)
    }
  }

  // 순찰 명령 함수
  const patrolCommand = async (selectedNodes, currentRobotSeq) => {
    if (selectedNodes.length < 2 || !currentRobotSeq) {
      console.warn('[RobotCommandsStore] 로봇이 선택되지 않았거나 경유지가 충분하지 않습니다.')
      return
    }
    try {
      const goals = selectedNodes.map(node => ({
        x: node.id[0],
        y: node.id[1],
        theta: 0.0
      }))
      await axios.post(
        `https://robocopbackendssafy.duckdns.org/api/v1/${currentRobotSeq}/call-service/patrol`,
        { goals }
      )
      console.log('[RobotCommandsStore] 순찰 명령 전송 완료')
    } catch (error) {
      console.error('Patrol request failed:', error)
    }
  }

  // 일시정지 명령 함수
  const tempStopCommand = async (currentRobotSeq) => {
    if (!currentRobotSeq) {
      console.warn('[RobotCommandsStore] 로봇이 선택되지 않았습니다.')
      return
    }
    try {
      await axios.post(
        `https://robocopbackendssafy.duckdns.org/api/v1/${currentRobotSeq}/call-service/temp-stop`
      )
      console.log('[RobotCommandsStore] 일시정지 요청 완료')
    } catch (error) {
      console.error('Temporary stop request failed:', error)
    }
  }

  // 선택 초기화 함수 (빈 배열 반환)
  const resetSelectionCommand = () => {
    return []
  }

  // 복귀 명령 함수 (homing)
  const homingCommand = async (currentRobotSeq) => {
    if (!currentRobotSeq) {
      console.warn('[RobotCommandsStore] 로봇이 선택되지 않았습니다.')
      return
    }
    try {
      await axios.post(
        `https://robocopbackendssafy.duckdns.org/api/v1/${currentRobotSeq}/call-service/homing`
      )
      console.log('[RobotCommandsStore] 복귀 명령 전송 완료')
    } catch (error) {
      console.error('Homing request failed:', error)
    }
  }

  // 비상 정지 명령 함수 (estop)
  const estopCommand = async (currentRobotSeq) => {
    if (!currentRobotSeq) {
      console.warn('[RobotCommandsStore] 로봇이 선택되지 않았습니다.')
      return
    }
    try {
      await axios.post(
        `https://robocopbackendssafy.duckdns.org/api/v1/${currentRobotSeq}/call-service/estop`
      )
      console.log('[RobotCommandsStore] 비상 정지 명령 전송 완료')
    } catch (error) {
      console.error('Estop request failed:', error)
    }
  }

  return {
    navigateCommand,
    patrolCommand,
    tempStopCommand,
    resetSelectionCommand,
    homingCommand,
    estopCommand
  }
})
