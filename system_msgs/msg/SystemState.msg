# 전체 시스템의 현재 상태
string system_state          # "NORMAL_OPERATION", "MARKER_DETECTED", "EXECUTING_PICKUP", "SYSTEM_ERROR"
string agv_state            # "NAVIGATING", "DECELERATING", "STOPPED", "RESUMING"
string arm_state            # "IDLE", "PLANNING", "EXECUTING", "COMPLETED"
bool agv_available          # AGV가 작업 가능한 상태인지
bool arm_available          # 로봇팔이 작업 가능한 상태인지
uint32 current_marker_id    # 현재 처리 중인 마커 ID (0이면 없음)
time timestamp

# 추가 제안:
float32 system_uptime       # 시스템 가동 시간
string last_error          # 마지막 오류 메시지
