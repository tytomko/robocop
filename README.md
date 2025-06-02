# 로봇 모니터링 시스템
| 프로젝트 기간 (2025.01.13 ~ 2025.02.21 / 6주)
| 서영은(팀장), 권우현, 권오준, 김성훈, 고태연, 신현학

## 프로젝트 클론 및 실행 방법
### Git 클론
```bash
# 프론트엔드
git clone https://github.com/fridec13/robocop-frontend.git
cd robocop-frontend


### 로컬 실행

#### Backend
1. Python 3.8 이상 설치
2. 가상환경 생성 및 활성화
```bash
cd robot-monitoring-backend
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate
```

3. 필요한 패키지 설치
```bash
pip install -r requirements.txt
```

4. 서버 실행
```bash
python main.py
# 서버가 http://localhost:8080 에서 실행됩니다
```

#### Frontend
1. Node.js 16 이상 설치
2. 프로젝트 설치 및 실행
```bash
cd robot-monitoring-frontend
npm install
npm run dev
# 프론트엔드가 http://localhost:5173 에서 실행됩니다
```

3. Vue 개발자 도구
- 프로젝트에 이미 `vite-plugin-vue-devtools`가 설치되어 있습니다
- 개발 서버 실행 시 자동으로 활성화됩니다
- 브라우저에서 `http://localhost:5173/__devtools__`로 접근하거나
- 개발 서버 실행 중 화면 우측 하단의 Vue 로고를 클릭하여 사용할 수 있습니다

## 시스템 구성
- Frontend: Vue.js 3 (Composition API)
- Backend: FastAPI
- 실시간 통신: WebSocket

## API 문서

### REST API 엔드포인트

#### 1. 인증 관련
- POST `/token`
  - 설명: 사용자 로그인
  - Request Body:
    ```json
    {
      "username": "string",
      "password": "string"
    }
    ```
  - Response:
    ```json
    {
      "access_token": "string",
      "token_type": "bearer"
    }
    ```

#### 2. 로봇 관리
- GET `/api/robots`
  - 설명: 전체 로봇 목록 조회
  - Response:
    ```json
    [
      {
        "id": "ROBOT_001",
        "name": "Robot 1",
        "status": "active",
        "battery": 85,
        "location": {
          "x": 23.5,
          "y": 45.2
        },
        "current_task": "patrol",
        "last_updated": "2023-12-20T14:30:00"
      }
    ]
    ```

- POST `/admin/robots`
  - 설명: 새 로봇 등록
  - Request Body:
    ```json
    {
      "robot_id": "string",
      "name": "string"
    }
    ```

#### 3. 카메라 상태
- GET `/api/camera/status`
  - 설명: 카메라 연결 상태 확인
  - Response:
    ```json
    {
      "status": "ok|error",
      "message": "string"
    }
    ```

### WebSocket 엔드포인트

#### 1. 카메라 스트림
- 엔드포인트: `ws://localhost:8080/ws/camera/{robot_id}`
- 설명: 실시간 카메라 영상 스트림
- 클라이언트 -> 서버 메시지:
  ```json
  {
    "action": "start_stream",
    "resolution": {
      "width": 640,
      "height": 480
    }
  }
  ```
- 서버 -> 클라이언트 메시지:
  ```json
  {
    "status": "streaming|error",
    "image": "base64_encoded_string",
    "timestamp": "2023-12-20T14:30:00"
  }
  ```

#### 2. 모니터링 데이터
- 엔드포인트: `ws://localhost:8080/ws/monitoring/{robot_id}`
- 설명: 실시간 로봇 상태 모니터링
- 서버 -> 클라이언트 메시지:
  ```json
  {
    "id": "ROBOT_001",
    "status": "active|charging|idle|error",
    "battery": 85,
    "location": {
      "x": 23.5,
      "y": 45.2
    },
    "current_task": "patrol|delivery|cleaning",
    "last_updated": "2023-12-20T14:30:00"
  }
  ```

#### 3. 센서 데이터
- 엔드포인트: `ws://localhost:8080/ws/sensor/{robot_id}`
- 설명: 실시간 센서 데이터 스트림
- 서버 -> 클라이언트 메시지:
  ```json
  {
    "robot_id": "ROBOT_001",
    "timestamp": "2023-12-20T14:30:00",
    "lidar_data": {
      "points": [
        {"x": 1.2, "y": 2.3}
      ]
    },
    "imu_data": {
      "acceleration": {"x": 0.1, "y": 0.2, "z": 9.8},
      "gyro": {"x": 0.01, "y": 0.02, "z": 0.03}
    },
    "position": {
      "x": 23.5,
      "y": 45.2,
      "orientation": 180.5
    }
  }
  ```

## 프론트엔드 컴포넌트 구조

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

## 에러 코드 및 메시지
### WebSocket 에러 코드
- 1000: 정상 종료
- 1001: 엔드포인트 종료
- 1002: 프로토콜 에러
- 1003: 데이터 타입 에러
- 1006: 비정상 종료
- 1015: TLS 핸드셰이크 실패

### 카메라 에러
- NotReadableError: 카메라가 다른 프로그램에서 사용 중
- NotAllowedError: 카메라 권한 거부됨
- NotFoundError: 카메라를 찾을 수 없음

## 데이터베이스 스키마

### 1. Users
```sql
CREATE TABLE users (
  username VARCHAR(50) PRIMARY KEY,
  email VARCHAR(100) UNIQUE,
  password_hash VARCHAR(100),
  is_active BOOLEAN DEFAULT true,
  is_admin BOOLEAN DEFAULT false
);
```

### 2. Robots
```sql
CREATE TABLE robots (
  id VARCHAR(50) PRIMARY KEY,
  name VARCHAR(100),
  status VARCHAR(20),
  battery_level INTEGER,
  last_updated TIMESTAMP
);
```

## 프론트엔드-백엔드 통신 가이드

### 1. HTTP 통신 (REST API)

#### Axios 설정
```javascript
// frontend/src/utils/axios.js
import axios from 'axios';

const api = axios.create({
  baseURL: 'http://localhost:8080',
  timeout: 5000,
  headers: {
    'Content-Type': 'application/json'
  }
});

// 인터셉터 설정
api.interceptors.request.use(
  config => {
    const token = localStorage.getItem('token');
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  error => Promise.reject(error)
);

export default api;
```

#### API 호출 예시
```javascript
// 로그인
const login = async (username, password) => {
  try {
    const response = await api.post('/token', { username, password });
    localStorage.setItem('token', response.data.access_token);
    return response.data;
  } catch (error) {
    console.error('로그인 실패:', error);
    throw error;
  }
};

// 로봇 목록 조회
const getRobots = async () => {
  try {
    const response = await api.get('/api/robots');
    return response.data;
  } catch (error) {
    console.error('로봇 목록 조회 실패:', error);
    throw error;
  }
};
```

### 2. WebSocket 통신

#### WebSocket 연결 관리
```javascript
// frontend/src/utils/websocket.js
class WebSocketManager {
  constructor(url, options = {}) {
    this.url = url;
    this.options = options;
    this.ws = null;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 3;
  }

  connect() {
    try {
      this.ws = new WebSocket(this.url);
      
      this.ws.onopen = () => {
        console.log('WebSocket 연결됨');
        this.reconnectAttempts = 0;
        if (this.options.onOpen) {
          this.options.onOpen();
        }
      };

      this.ws.onmessage = (event) => {
        if (this.options.onMessage) {
          this.options.onMessage(event);
        }
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket 에러:', error);
        if (this.options.onError) {
          this.options.onError(error);
        }
      };

      this.ws.onclose = (event) => {
        console.log('WebSocket 연결 종료:', event);
        if (!event.wasClean && this.reconnectAttempts < this.maxReconnectAttempts) {
          this.reconnectAttempts++;
          setTimeout(() => this.connect(), 3000);
        }
        if (this.options.onClose) {
          this.options.onClose(event);
        }
      };
    } catch (error) {
      console.error('WebSocket 연결 실패:', error);
    }
  }

  send(data) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    } else {
      console.error('WebSocket이 연결되어 있지 않습니다');
    }
  }

  close() {
    if (this.ws) {
      this.ws.close(1000, '사용자가 연결을 종료했습니다');
    }
  }
}
```

#### WebSocket 사용 예시
```javascript
// 카메라 스트림 연결
const connectCamera = (robotId) => {
  const wsCamera = new WebSocketManager(
    `ws://localhost:8080/ws/camera/${robotId}`,
    {
      onOpen: () => {
        console.log('카메라 스트림 연결됨');
        wsCamera.send({
          action: 'start_stream',
          resolution: { width: 640, height: 480 }
        });
      },
      onMessage: (event) => {
        try {
          if (event.data instanceof Blob) {
            // 바이너리 데이터 처리
            const reader = new FileReader();
            reader.onload = () => {
              const base64data = reader.result.split(',')[1];
              updateCameraView(base64data);
            };
            reader.readAsDataURL(event.data);
          } else {
            // JSON 메시지 처리
            const data = JSON.parse(event.data);
            handleCameraStatus(data);
          }
        } catch (error) {
          console.error('카메라 데이터 처리 실패:', error);
        }
      },
      onError: (error) => {
        console.error('카메라 스트림 에러:', error);
      }
    }
  );
  return wsCamera;
};

// 모니터링 데이터 연결
const connectMonitoring = (robotId) => {
  const wsMonitoring = new WebSocketManager(
    `ws://localhost:8080/ws/monitoring/${robotId}`,
    {
      onMessage: (event) => {
        try {
          const data = JSON.parse(event.data);
          updateRobotStatus(data);
        } catch (error) {
          console.error('모니터링 데이터 처리 실패:', error);
        }
      }
    }
  );
  return wsMonitoring;
};
```

### 3. 데이터 흐름

#### 컴포넌트에서의 사용
```javascript
// CameraView.vue
import { ref, onMounted, onUnmounted } from 'vue';
import WebSocketManager from '@/utils/websocket';

export default {
  setup() {
    const wsCamera = ref(null);
    
    onMounted(() => {
      wsCamera.value = connectCamera('ROBOT_001');
    });
    
    onUnmounted(() => {
      if (wsCamera.value) {
        wsCamera.value.close();
      }
    });
  }
};
```

### 4. 에러 처리 및 재시도 전략

#### HTTP 요청 재시도
```javascript
const retryRequest = async (fn, maxRetries = 3) => {
  let attempts = 0;
  while (attempts < maxRetries) {
    try {
      return await fn();
    } catch (error) {
      attempts++;
      if (attempts === maxRetries) throw error;
      await new Promise(resolve => setTimeout(resolve, 1000 * attempts));
    }
  }
};
```

#### WebSocket 재연결
- 연결 실패 시 최대 3회 재시도
- 지수 백오프 방식으로 재연결 간격 증가
- 재연결 시 이전 연결 상태 복구

### 5. 보안 고려사항

#### 토큰 관리
- 액세스 토큰은 localStorage에 저장
- WebSocket 연결 시 토큰을 쿼리 파라미터로 전달
- 토큰 만료 시 자동 재로그인

#### CORS 설정
- 개발 환경: 모든 오리진 허용
- 프로덕션 환경: 특정 도메인만 허용 

# RoboCop Frontend

## JWT 인증 구현 가이드

### 1. 토큰 정보
- **Access Token**
  - 유효 시간: 3시간 (180분)
  - 용도: API 요청 인증
  - 저장 위치: localStorage 또는 secure cookie

- **Refresh Token**
  - 유효 시간: 6시간 (360분)
  - 용도: Access Token 갱신
  - 저장 위치: localStorage 또는 secure cookie

### 2. API 엔드포인트
```typescript
const API_BASE_URL = '/api/v1/auth';

// 로그인
POST ${API_BASE_URL}/login
Request:
{
  username: string,
  password: string
}
Response:
{
  access_token: string,
  refresh_token: string,
  token_type: "bearer"
}

// 토큰 갱신
POST ${API_BASE_URL}/refresh
Request:
{
  current_token: string // refresh_token
}
Response:
{
  access_token: string,
  refresh_token: string,
  token_type: "bearer"
}

// 로그아웃
POST ${API_BASE_URL}/logout
Headers:
{
  Authorization: "Bearer ${access_token}"
}
```

### 3. 프론트엔드 구현 예시

```typescript
// src/services/TokenService.ts
import jwtDecode from 'jwt-decode';
import axios from 'axios';

interface TokenPayload {
  exp: number;
  sub: string;
}

class TokenService {
  private readonly REFRESH_THRESHOLD = 15 * 60 * 1000; // 15분
  private readonly API_BASE_URL = '/api/v1/auth';

  // 토큰 저장
  setTokens(accessToken: string, refreshToken: string) {
    localStorage.setItem('accessToken', accessToken);
    localStorage.setItem('refreshToken', refreshToken);
  }

  // 토큰 제거
  clearTokens() {
    localStorage.removeItem('accessToken');
    localStorage.removeItem('refreshToken');
  }

  // 토큰 만료 확인
  private isTokenExpiringSoon(token: string): boolean {
    try {
      const decoded = jwtDecode<TokenPayload>(token);
      const expiryTime = decoded.exp * 1000;
      return Date.now() + this.REFRESH_THRESHOLD > expiryTime;
    } catch {
      return true;
    }
  }

  // 토큰 갱신
  private async refreshTokens(): Promise<boolean> {
    try {
      const refreshToken = localStorage.getItem('refreshToken');
      if (!refreshToken) return false;

      const response = await axios.post(`${this.API_BASE_URL}/refresh`, {
        current_token: refreshToken
      });

      const { access_token, refresh_token } = response.data;
      this.setTokens(access_token, refresh_token);
      return true;
    } catch {
      this.clearTokens();
      return false;
    }
  }

  // API 요청 전 토큰 유효성 확인
  async ensureValidToken(): Promise<boolean> {
    const accessToken = localStorage.getItem('accessToken');
    
    if (!accessToken) {
      return false;
    }

    if (this.isTokenExpiringSoon(accessToken)) {
      return await this.refreshTokens();
    }

    return true;
  }

  // Axios 인터셉터 설정
  setupInterceptors() {
    axios.interceptors.request.use(
      async (config) => {
        if (await this.ensureValidToken()) {
          const accessToken = localStorage.getItem('accessToken');
          config.headers.Authorization = `Bearer ${accessToken}`;
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    axios.interceptors.response.use(
      (response) => response,
      async (error) => {
        if (error.response?.status === 401) {
          if (await this.refreshTokens()) {
            // 실패한 요청 재시도
            const accessToken = localStorage.getItem('accessToken');
            error.config.headers.Authorization = `Bearer ${accessToken}`;
            return axios(error.config);
          }
        }
        return Promise.reject(error);
      }
    );
  }
}

export const tokenService = new TokenService();
```

### 4. 사용 방법

```typescript
// src/App.tsx 또는 진입점
import { tokenService } from './services/TokenService';

// 앱 시작 시 인터셉터 설정
tokenService.setupInterceptors();

// API 요청 시 자동으로 토큰 처리됨
async function fetchData() {
  try {
    const response = await axios.get('/api/data');
    // 처리 로직
  } catch (error) {
    // 에러 처리
  }
}
```

### 5. 주의사항

1. **토큰 저장**
   - 보안을 위해 httpOnly 쿠키 사용 고려
   - XSS 공격 방지를 위한 조치 필요

2. **에러 처리**
   - 401 에러: 토큰 만료
   - 403 에러: 권한 없음
   - Refresh 실패: 로그인 페이지로 리다이렉트

3. **보안**
   - HTTPS 사용 필수
   - CSRF 토큰 구현 고려
   - 민감한 정보는 암호화하여 저장