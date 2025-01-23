<template>
  <div class="course-management">
    <div class="page-header">
      <h2>코스 관리</h2>
      <div class="header-actions">
        <button class="action-btn" @click="startCourseEdit">
          <i class="fas fa-plus"></i> 새 코스 생성
        </button>
        <button class="action-btn" @click="saveCurrentCourse" :disabled="!isEditing">
          <i class="fas fa-save"></i> 코스 저장
        </button>
      </div>
    </div>

    <div class="content-layout">
      <!-- 왼쪽: 지도 및 경로 편집 -->
      <div class="map-section">
        <MapView 
          ref="mapView"
          :map-data="mapData" 
          :robot-position="robotPosition"
          :path-points="currentCourse.points"
          :show-markers="true"
          @map-click="handleMapClick"
        />
        <div class="map-controls" v-if="isEditing">
          <div class="control-group">
            <button class="control-btn" @click="setStartPoint" :class="{ active: editMode === 'start' }">
              시작점 설정
            </button>
            <button class="control-btn" @click="setEndPoint" :class="{ active: editMode === 'end' }">
              도착점 설정
            </button>
            <button class="control-btn" @click="addWaypoint" :class="{ active: editMode === 'waypoint' }">
              경유점 추가
            </button>
          </div>
          <div class="control-group">
            <button class="control-btn danger" @click="clearPoints">
              초기화
            </button>
          </div>
        </div>
      </div>

      <!-- 오른쪽: 코스 목록 및 상세 정보 -->
      <div class="course-list-section">
        <div class="section-header">
          <h3>저장된 코스</h3>
        </div>
        <div class="course-list">
          <div 
            v-for="course in savedCourses" 
            :key="course.id"
            class="course-item"
            :class="{ active: selectedCourseId === course.id }"
            @click="selectCourse(course)"
          >
            <div class="course-info">
              <h4>{{ course.name }}</h4>
              <p>{{ course.description }}</p>
              <div class="course-stats">
                <span>거리: {{ course.distance }}m</span>
                <span>경유점: {{ course.points.length - 2 }}개</span>
              </div>
            </div>
            <div class="course-actions">
              <button class="icon-btn" @click.stop="startCourse(course)">
                <i class="fas fa-play"></i>
              </button>
              <button class="icon-btn" @click.stop="editCourse(course)">
                <i class="fas fa-edit"></i>
              </button>
              <button class="icon-btn danger" @click.stop="deleteCourse(course)">
                <i class="fas fa-trash"></i>
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 코스 편집 모달 -->
    <div class="modal" v-if="showEditModal" @click.self="closeEditModal">
      <div class="modal-content">
        <h3>{{ isEditingExisting ? '코스 수정' : '새 코스 생성' }}</h3>
        <form @submit.prevent="saveCourse">
          <div class="form-group">
            <label>코스명</label>
            <input v-model="currentCourse.name" type="text" required>
          </div>
          <div class="form-group">
            <label>설명</label>
            <textarea v-model="currentCourse.description" rows="3"></textarea>
          </div>
          <div class="modal-actions">
            <button type="button" class="cancel-btn" @click="closeEditModal">취소</button>
            <button type="submit" class="submit-btn">저장</button>
          </div>
        </form>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import MapView from '@/components/map/MapView.vue'
import { mapService } from '@/services/mapService'
import { webSocketService } from '@/services/websocket'

// 상태 관리
const mapData = ref(null)
const robotPosition = ref({ x: 0, y: 0, theta: 0 })
const isEditing = ref(false)
const editMode = ref(null) // 'start', 'end', 'waypoint'
const showEditModal = ref(false)
const isEditingExisting = ref(false)
const selectedCourseId = ref(null)

// 현재 편집 중인 코스
const currentCourse = ref({
  id: null,
  name: '',
  description: '',
  points: [],
  distance: 0
})

// 저장된 코스 목록 (임시 데이터)
const savedCourses = ref([
  {
    id: 1,
    name: '순찰 코스 1',
    description: '1층 주요 구역 순찰',
    points: [
      { x: 100, y: 100, type: 'start' },
      { x: 200, y: 200, type: 'waypoint' },
      { x: 300, y: 300, type: 'end' }
    ],
    distance: 50
  }
])

// 지도 데이터 로드
const loadMapData = async () => {
  try {
    const loadedMapData = await mapService.loadMap(
      '/maps/current_map.png',
      '/maps/current_map.json'
    )
    if (!loadedMapData) {
      console.error('지도 데이터를 불러올 수 없습니다.')
      return
    }
    mapData.value = loadedMapData
  } catch (error) {
    console.error('지도 로드 실패:', error)
  }
}

// 웹소켓 설정
const setupWebSocket = async () => {
  try {
    await webSocketService.connect('ws://localhost:8000/ws/robot')
    
    webSocketService.subscribe('robot_position', (data) => {
      if (!data || !mapData.value) return
      
      try {
        const pixelPos = mapService.worldToPixel(
          data.x ?? 0,
          data.y ?? 0,
          mapData.value
        )
        
        robotPosition.value = {
          x: pixelPos?.x ?? 0,
          y: pixelPos?.y ?? 0,
          theta: data.theta ?? 0
        }
      } catch (error) {
        console.error('로봇 위치 변환 실패:', error)
      }
    })
  } catch (error) {
    console.error('웹소켓 연결 실패:', error)
  }
}

// 지도 클릭 핸들러
const handleMapClick = (point) => {
  if (!isEditing.value || !editMode.value || !mapData.value || !point) return

  try {
    const worldPoint = mapService.pixelToWorld(point.x ?? 0, point.y ?? 0, mapData.value)
    if (!worldPoint) {
      console.warn('좌표 변환 실패')
      return
    }
    
    switch (editMode.value) {
      case 'start':
        if (!currentCourse.value.points) currentCourse.value.points = []
        currentCourse.value.points = [{ ...worldPoint, type: 'start' }, ...(currentCourse.value.points.slice(1) || [])]
        break
      case 'end':
        if (!currentCourse.value.points) currentCourse.value.points = []
        currentCourse.value.points = [...(currentCourse.value.points.slice(0, -1) || []), { ...worldPoint, type: 'end' }]
        break
      case 'waypoint':
        if (!currentCourse.value.points) currentCourse.value.points = []
        currentCourse.value.points.splice(-1, 0, { ...worldPoint, type: 'waypoint' })
        break
    }
    
    updateCourseDistance()
  } catch (error) {
    console.error('지도 클릭 처리 실패:', error)
  }
}

// 코스 거리 계산
const updateCourseDistance = () => {
  if (!currentCourse.value || !currentCourse.value.points) {
    currentCourse.value = {
      ...currentCourse.value,
      distance: 0,
      points: []
    }
    return
  }

  let distance = 0
  const points = currentCourse.value.points
  
  for (let i = 1; i < points.length; i++) {
    if (!points[i] || !points[i-1]) continue
    
    const dx = (points[i].x ?? 0) - (points[i-1].x ?? 0)
    const dy = (points[i].y ?? 0) - (points[i-1].y ?? 0)
    distance += Math.sqrt(dx * dx + dy * dy)
  }
  
  currentCourse.value.distance = Math.round(distance * 100) / 100
}

// 편집 모드 제어
const startCourseEdit = () => {
  isEditing.value = true
  isEditingExisting.value = false
  showEditModal.value = true
  currentCourse.value = {
    id: null,
    name: '',
    description: '',
    points: [],
    distance: 0
  }
}

const editCourse = (course) => {
  if (!course) return
  
  isEditing.value = true
  isEditingExisting.value = true
  showEditModal.value = true
  currentCourse.value = { 
    ...course,
    points: Array.isArray(course.points) ? [...course.points] : []
  }
  selectedCourseId.value = course.id
}

const closeEditModal = () => {
  showEditModal.value = false
  if (!isEditingExisting.value) {
    isEditing.value = false
    currentCourse.value = {
      id: null,
      name: '',
      description: '',
      points: [],
      distance: 0
    }
  }
}

// 포인트 설정
const setStartPoint = () => {
  editMode.value = 'start'
}

const setEndPoint = () => {
  editMode.value = 'end'
}

const addWaypoint = () => {
  editMode.value = 'waypoint'
}

const clearPoints = () => {
  currentCourse.value.points = []
  currentCourse.value.distance = 0
}

// 코스 관리
const saveCourse = () => {
  if (currentCourse.value.points.length < 2) {
    alert('시작점과 도착점을 모두 설정해주세요.')
    return
  }

  if (isEditingExisting.value) {
    const index = savedCourses.value.findIndex(c => c.id === currentCourse.value.id)
    if (index !== -1) {
      savedCourses.value[index] = { ...currentCourse.value }
    }
  } else {
    currentCourse.value.id = savedCourses.value.length + 1
    savedCourses.value.push({ ...currentCourse.value })
  }

  closeEditModal()
}

const selectCourse = (course) => {
  selectedCourseId.value = course.id
  currentCourse.value = { ...course }
}

const startCourse = (course) => {
  webSocketService.send('start_course', {
    courseId: course.id,
    points: course.points
  })
}

const deleteCourse = (course) => {
  if (confirm('정말 삭제하시겠습니까?')) {
    savedCourses.value = savedCourses.value.filter(c => c.id !== course.id)
    if (selectedCourseId.value === course.id) {
      selectedCourseId.value = null
    }
  }
}

// 라이프사이클 훅
onMounted(async () => {
  await loadMapData()
  await setupWebSocket()
})

onUnmounted(() => {
  webSocketService.disconnect()
})
</script>

<style scoped>
.course-management {
  padding: 20px;
  height: 100%;
  display: flex;
  flex-direction: column;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.header-actions {
  display: flex;
  gap: 10px;
}

.content-layout {
  display: grid;
  grid-template-columns: 1fr 300px;
  gap: 20px;
  flex: 1;
  min-height: 0;
}

.map-section {
  position: relative;
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  overflow: hidden;
}

.map-controls {
  position: absolute;
  top: 20px;
  right: 20px;
  background: white;
  padding: 10px;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.control-group {
  display: flex;
  gap: 5px;
}

.course-list-section {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  display: flex;
  flex-direction: column;
}

.section-header {
  padding: 15px;
  border-bottom: 1px solid #eee;
}

.section-header h3 {
  margin: 0;
}

.course-list {
  flex: 1;
  overflow-y: auto;
  padding: 10px;
}

.course-item {
  background: #f8f9fa;
  border-radius: 8px;
  padding: 15px;
  margin-bottom: 10px;
  cursor: pointer;
  transition: all 0.3s ease;
}

.course-item:hover {
  background: #e9ecef;
}

.course-item.active {
  background: #e3f2fd;
}

.course-info h4 {
  margin: 0 0 5px 0;
}

.course-info p {
  margin: 0 0 10px 0;
  color: #666;
  font-size: 0.9em;
}

.course-stats {
  display: flex;
  gap: 15px;
  font-size: 0.85em;
  color: #666;
}

.course-actions {
  display: flex;
  justify-content: flex-end;
  gap: 5px;
  margin-top: 10px;
}

.action-btn {
  padding: 8px 16px;
  border: none;
  border-radius: 4px;
  background: #2196F3;
  color: white;
  cursor: pointer;
  display: flex;
  align-items: center;
  gap: 5px;
}

.action-btn:hover {
  background: #1976D2;
}

.control-btn {
  padding: 8px 16px;
  border: 1px solid #ddd;
  border-radius: 4px;
  background: white;
  cursor: pointer;
}

.control-btn.active {
  background: #2196F3;
  color: white;
  border-color: #2196F3;
}

.control-btn.danger {
  color: #dc3545;
  border-color: #dc3545;
}

.icon-btn {
  width: 30px;
  height: 30px;
  border: none;
  border-radius: 4px;
  background: #e9ecef;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
}

.icon-btn:hover {
  background: #dee2e6;
}

.icon-btn.danger {
  color: #dc3545;
}

.icon-btn.danger:hover {
  background: #dc3545;
  color: white;
}

/* 모달 스타일 */
.modal {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.modal-content {
  background: white;
  padding: 20px;
  border-radius: 8px;
  width: 100%;
  max-width: 400px;
}

.form-group {
  margin-bottom: 15px;
}

.form-group label {
  display: block;
  margin-bottom: 5px;
  color: #333;
}

.form-group input,
.form-group textarea {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
}

.modal-actions {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
  margin-top: 20px;
}

.cancel-btn {
  padding: 8px 16px;
  border: 1px solid #ddd;
  background: white;
  border-radius: 4px;
  cursor: pointer;
}

.submit-btn {
  padding: 8px 16px;
  background: #4CAF50;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.submit-btn:hover {
  background: #45a049;
}
</style> 