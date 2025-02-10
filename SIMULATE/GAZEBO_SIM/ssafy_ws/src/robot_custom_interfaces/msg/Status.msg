# 로봇의 고유 ID
int32 id

# 로봇의 이름
string name

# 현재 상태를 나타내는 모드 
# emergency stop / homing / navigating / patrol / waiting
string mode

# 배터리 (더미)
float32 battery

# 로봇의 CPU온도(섭씨)(더미)
float32 temperatures

# 네트워크 상테 (더미)
float32 network

# 작동 시작시간
string starttime

# true면 ON, false면 OFF
bool is_active
