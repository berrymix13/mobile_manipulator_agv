# AGV 네비게이션 상태 정보
string status               # "NAVIGATING", "PLANNING", "DECELERATING", "STOPPED", "RESUMING", "ERROR"
geometry_msgs/Pose current_pose
geometry_msgs/Pose target_pose
float32 current_speed       # 현재 속도 (m/s)
float32 distance_to_goal    # 목표까지 거리 (m)
bool path_blocked          # 경로 차단 여부
bool goal_reached          # 목표 도달 여부
time timestamp
