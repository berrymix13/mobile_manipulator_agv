# 마커 감지 정보
uint32 marker_id
geometry_msgs/PoseStamped pose    # 마커의 3D 위치
float32 distance                 # 로봇까지의 거리
bool in_work_range              # 작업 범위 내에 있는지
time detection_time

# 추가 제안:
float32 marker_size         # 마커 크기 (m)
string marker_type         # "ARUCO", "QR", etc.