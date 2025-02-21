`# 로봇 관제 시스템 구현 현황

## 구현된 컴포넌트

### 1. 대시보드 레이아웃
- [x] Navbar 구현 
- [x] 좌측 사이드바 로봇 리스트 (ListSidebarSection.vue)

### 2. 현황 섹션
- [x] 메인 레이아웃 (MonitoringView.vue)
  - [x] 실시간 로봇 위치 (RobotMap.vue)
    - [x] 전체 로봇 위치 확인
  - [x] 등록된 로봇 목록 (RobotList.vue)
    - [x] 로봇 상세 페이지 (RobotDetailView.vue)
    - [x] 로봇 관리 모달 (RobotManagement.vue)
    - [x] 로봇 등록 모달 (RobotRegistration.vue)
  - [x] 통계 (StatisticsView.vue)
    - [x] 통계 메트릭 박스 (StatsMetricBox.vue)
    - [x] 통계 차트 (StatsChart.vue)
    - [x] 통계 테이블 (StatsTable.vue)

### 3. CCTV 섹션
- [x] 메인 레이아웃 (CameraView.vue)
  - [x] 로봇 선택 기능
  - [x] 최대 2개 로봇 동시 송출(Cctv.vue)
  - [x] 카메라 방향 설정

### 4. 제어 페이지 섹션
- [x] 메인 레이아웃 (RobotControlView.vue)
  - [x] 선택 로봇 및 활동중인 로봇 확인 (RobotMap.vue)
  - [x] 자동/수동 모드 전환 기능 
  - [x] 경로 선택 후 이동 명령 (ControlButtons.vue, SelectedNodes.vue)
  - [x] 수동 제어

### 5. 관리 페이지 섹션
- [x] 메인 레이아웃 (ManagementView.vue)
  - [x] 사용자 관리
    - [x] 비밀번호 변경 (PasswordChange.vue)
    - [x] 로그아웃
  - [x] 등록자 관리(EnrollmentView.vue)
    - [x] 등록된 사용자 리스트 (UserList.vue)
    - [x] 사용자 등록 모달 (EnrollModal.vue)
    - [x] 이미지 업로드를 통한 얼굴 데이터 학습