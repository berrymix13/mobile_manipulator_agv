#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from system_msgs.msg import MarkerInfo
from system_msgs.msg import MarkerApproachAction, MarkerApproachGoal

# State Machine
class AGVState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    SLOWING_DOWN = "slowing_down"
    APPROACHING_MARKER = "approaching_marker"  
    STOPPED_FOR_WORK = "stopped_for_work"
    WORKING = "working"
    RETURNING = "returning"
    COMPLETED = "completed"

class AGVController:
    def __init__(self):
        rospy.init_node('agv_controller', anonymous=False)
        
        # AGV 상태 관리
        self.current_state = AGVState.IDLE
        self.current_waypoint_index = 0
        self.marker_detected = False
        self.work_completed = False
        self.work_requested = False  # 작업 요청 플래그 추가
        self.work_start_time = None  # 작업 시작 시간 추적
        self.completed_markers = set()  # 완료된 마커 ID 추적
        
        # 파라미터 설정
        self.marker_stop_distance = rospy.get_param('~marker_stop_distance', 0.3)  # 30cm (기존 26cm에서 증가)
        self.marker_slow_distance = rospy.get_param('~marker_slow_distance', 1.0)  # 1m
        self.max_approach_distance = rospy.get_param('~max_approach_distance', 1.0)  # 1m
        self.slow_speed = rospy.get_param('~slow_speed', 0.05)  # 감속 시 속도 (m/s) - 더 느리게
        self.work_timeout = rospy.get_param('~work_timeout', 30.0)  # 작업 타임아웃 (초)
        
        # Waypoint 리스트 (시작점 포함)
        self.waypoints = self._load_waypoints()
        self.home_position = self.waypoints[0]  # 첫 번째 waypoint를 홈으로 설정
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=1)
        
        # Subscribers  
        self.marker_info_sub = rospy.Subscriber('/marker_info', MarkerInfo, self.marker_info_callback)
        self.arm_status_sub = rospy.Subscriber('/arm_work_status', String, self.arm_status_callback)
        
        # Arm Action Clients
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.marker_approach_client = actionlib.SimpleActionClient('marker_approach', MarkerApproachAction)
        
        # Action Client 연결 대기
        rospy.loginfo("AGV Controller: Waiting for move_base action server...")
        self.move_base_client.wait_for_server(timeout=rospy.Duration(30.0))
        
        rospy.loginfo("AGV Controller: Waiting for marker_approach action server...")
        self.marker_approach_client.wait_for_server(timeout=rospy.Duration(30.0))
        
        rospy.loginfo("AGV Controller: All action servers connected!")
        
        # 타이머 설정
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        # 초기 상태 발행
        self._publish_status("AGV Controller initialized")

    def _load_waypoints(self):
        """waypoint 리스트 로드 (실제 환경에 맞게 수정 필요)"""
        waypoints = []
        
        # 시작점 (홈 포지션) - navigation_active2.launch의 initial_pose와 동일
        home_pose = PoseStamped()
        home_pose.header.frame_id = "map"
        home_pose.pose.position.x = 0.6
        home_pose.pose.position.y = 0.1
        home_pose.pose.position.z = 0.0
        home_pose.pose.orientation.w = 1.0
        waypoints.append(home_pose)
        
        # 첫 번째 목표점
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = "map"
        waypoint1.pose.position.x = 0.82
        waypoint1.pose.position.y = -0.131
        waypoint1.pose.position.z = -0.00143
        waypoint1.pose.orientation.w = 1.0
        waypoints.append(waypoint1)
        
        # 두 번째 목표점 (주석처리)
        # waypoint2 = PoseStamped()
        # waypoint2.header.frame_id = "map"
        # waypoint2.pose.position.x = 3.0
        # waypoint2.pose.position.y = 2.0
        # waypoint2.pose.position.z = 0.0
        # waypoint2.pose.orientation.w = 1.0
        # waypoints.append(waypoint2)
        
        rospy.loginfo(f"AGV Controller: Loaded {len(waypoints)} waypoints")
        return waypoints

    def marker_info_callback(self, msg):
        """마커 정보 콜백"""
        if msg.marker_id >= 0:  # 유효한 마커가 감지됨
            self.marker_detected = True
            self.current_marker_distance = msg.distance
            self.current_marker_info = msg
            
            # 마커 정보 로깅 (간소화)
            rospy.logdebug(f"AGV Controller: Marker {msg.marker_id} detected - "
                          f"Distance: {msg.distance:.3f}m, "
                          f"Type: {msg.marker_type}, "
                          f"Size: {msg.marker_size:.3f}m, "
                          f"In work range: {msg.in_work_range}")
            
            # 마커 감지 시 현재 상태 확인
            if self.current_state in [AGVState.NAVIGATING, AGVState.SLOWING_DOWN]:
                # 이미 완료된 마커인지 확인
                if msg.marker_id in self.completed_markers:
                    rospy.logdebug(f"AGV Controller: Marker {msg.marker_id} already completed - Ignoring")
                    return
                
                # 작업 범위 내에 있는지 확인
                if msg.in_work_range and self.current_state == AGVState.NAVIGATING:
                    rospy.loginfo(f"AGV Controller: Marker {msg.marker_id} is in work range - Starting to slow down")
                    self.current_state = AGVState.SLOWING_DOWN
                    self._publish_status(f"Slowing down for marker {msg.marker_id}")
                
                # 마커가 감속 거리 이내에 있고 현재 주행 중인 경우
                elif (msg.distance <= self.marker_slow_distance and 
                      self.current_state == AGVState.NAVIGATING):
                    rospy.loginfo(f"AGV Controller: Marker {msg.marker_id} at {msg.distance:.3f}m - Starting to slow down")
                    self.current_state = AGVState.SLOWING_DOWN
                    self._publish_status(f"Slowing down for marker {msg.marker_id}")
                
                # 마커가 정지 거리 이내에 있고 감속 중인 경우
                elif (msg.distance <= self.marker_stop_distance and 
                      self.current_state == AGVState.SLOWING_DOWN):
                    rospy.loginfo(f"AGV Controller: Marker {msg.marker_id} at {msg.distance:.3f}m - Stopping AGV")
                    self._stop_agv()
                    self.current_state = AGVState.STOPPED_FOR_WORK
            else:
                # 다른 상태에서는 마커 정보만 저장하고 로깅
                rospy.logdebug(f"AGV Controller: Marker {msg.marker_id} detected but AGV is in {self.current_state.name} state - Ignoring")
        else:
            self.marker_detected = False
            rospy.loginfo("AGV Controller: No marker detected")

    def arm_status_callback(self, msg):
        """로봇팔 상태 콜백"""
        rospy.logdebug(f"AGV Controller: Received arm status: {msg.data}")
        
        if msg.data == "COMPLETED":
            # 로봇팔 작업 완료 - 다음 작업 진행
            rospy.loginfo("AGV Controller: Arm work completed via status topic")
            self.proceed_to_next_task()
        elif msg.data == "FAILED":
            # 로봇팔 작업 실패 - 오류 처리
            rospy.logwarn("AGV Controller: Arm work failed via status topic")
            self.handle_arm_error()
        elif msg.data == "WORKING":
            # 로봇팔 작업 중 - 대기
            rospy.logdebug("AGV Controller: Arm is working - Waiting for completion")
            self.wait_for_arm_completion()

    def control_loop(self, event):
        """메인 제어 루프"""
        if self.current_state == AGVState.IDLE:
            self._start_navigation()
            
        elif self.current_state == AGVState.NAVIGATING:
            self._check_navigation_status()
            
        elif self.current_state == AGVState.SLOWING_DOWN:
            self._slow_down_for_marker()
            
        elif self.current_state == AGVState.STOPPED_FOR_WORK:
            if not self.work_requested:
                self._request_arm_work()
            # work_requested가 True이면 아무것도 하지 않음 (WORKING 상태로 전환 대기)
            
        elif self.current_state == AGVState.WORKING:
            self._check_work_status()
            
        elif self.current_state == AGVState.RETURNING:
            self._check_return_status()

    def _start_navigation(self):
        """waypoint 기반 네비게이션 시작"""
        if self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
            
            goal = MoveBaseGoal()
            goal.target_pose = target
            goal.target_pose.header.stamp = rospy.Time.now()
            
            rospy.loginfo(f"AGV Controller: Navigating to waypoint {self.current_waypoint_index}")
            self.move_base_client.send_goal(goal)
            self.current_state = AGVState.NAVIGATING
            self.move_base_cancelled = False  # move_base 상태 리셋
            self.work_requested = False  # 작업 요청 플래그 리셋
            self._publish_status(f"Navigating to waypoint {self.current_waypoint_index}")

    def _check_navigation_status(self):
        """네비게이션 상태 확인"""
        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"AGV Controller: Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints):
                # 모든 waypoint 완료 - 홈으로 복귀
                self._return_home()
            else:
                # 다음 waypoint로 이동
                self.current_state = AGVState.IDLE
                
        elif self.move_base_client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("AGV Controller: Navigation failed - Retrying")
            self.current_state = AGVState.IDLE

    def _slow_down_for_marker(self):
        """마커를 위해 감속"""
        if self.marker_detected and hasattr(self, 'current_marker_distance'):
            # 마커가 정지 거리 이내에 도달한 경우
            if self.current_marker_distance <= self.marker_stop_distance:
                rospy.loginfo(f"AGV Controller: Marker at {self.current_marker_distance:.3f}m - Stopping AGV")
                self._stop_agv()
                self.current_state = AGVState.STOPPED_FOR_WORK
            else:
                # move_base를 일시 정지하고 감속 명령 발행
                if not hasattr(self, 'move_base_cancelled') or not self.move_base_cancelled:
                    rospy.loginfo(f"AGV Controller: Slowing down for marker at {self.current_marker_distance:.3f}m")
                    self.move_base_client.cancel_all_goals()
                    self.move_base_cancelled = True
                
                # 감속 명령 발행
                slow_cmd = Twist()
                slow_cmd.linear.x = self.slow_speed
                self.cmd_vel_pub.publish(slow_cmd)
        else:
            # 마커가 사라진 경우 정상 속도로 복귀
            rospy.loginfo("AGV Controller: Marker lost - Resuming normal navigation")
            self.move_base_cancelled = False
            self.current_state = AGVState.NAVIGATING

    def _stop_agv(self):
        """AGV 즉시 정지"""
        # move_base 목표 취소
        self.move_base_client.cancel_all_goals()
        
        # 속도 0으로 설정하고 지속적으로 발행
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        
        # 정지 명령을 여러 번 발행하여 확실히 정지
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_cmd)
            rospy.sleep(0.1)
        
        rospy.loginfo("AGV Controller: AGV stopped for marker work")
        self._publish_status("AGV stopped - preparing for arm work")

    def _request_arm_work(self):
        """로봇팔에게 작업 요청"""
        if hasattr(self, 'current_marker_info') and not self.work_requested:
            # 이미 완료된 마커인지 확인
            if self.current_marker_info.marker_id in self.completed_markers:
                rospy.loginfo(f"AGV Controller: Marker {self.current_marker_info.marker_id} already completed - Resuming navigation")
                self.work_requested = False
                self.move_base_cancelled = False
                self.current_state = AGVState.IDLE
                self._publish_status("Marker already completed - Resuming navigation")
                return
                
            goal = MarkerApproachGoal()
            goal.target_marker_id = self.current_marker_info.marker_id
            
            # 마커 크기에 따라 접근 거리 조정
            if self.current_marker_info.marker_size > 0:
                # 마커 크기의 1.5배를 접근 거리로 설정 (최소 0.2m, 최대 0.5m)
                approach_dist = max(0.2, min(0.5, self.current_marker_info.marker_size * 1.5))
            else:
                approach_dist = 0.3  # 기본값
            
            goal.approach_distance = approach_dist
            goal.approach_height = 0.1    # 10cm 접근 높이
            goal.return_to_home = True    # 작업 후 초기 자세로 복귀
            
            rospy.loginfo(f"AGV Controller: Requesting arm work for marker {goal.target_marker_id} "
                         f"(Type: {self.current_marker_info.marker_type}, "
                         f"Size: {self.current_marker_info.marker_size:.3f}m, "
                         f"Approach distance: {approach_dist:.3f}m)")
            
            self.marker_approach_client.send_goal(goal)
            self.work_requested = True  # 작업 요청 플래그 설정
            self.work_start_time = rospy.Time.now()  # 작업 시작 시간 기록
            self.current_state = AGVState.WORKING
            self._publish_status(f"Robot arm working on marker {goal.target_marker_id}...")

    def _check_work_status(self):
        """로봇팔 작업 상태 확인 (액션 서버 백업 방식)"""
        # 상태 토픽이 우선되므로, 액션 서버는 백업으로만 사용
        state = self.marker_approach_client.get_state()
        
        # 타임아웃 체크
        if self.work_start_time is not None:
            elapsed_time = (rospy.Time.now() - self.work_start_time).to_sec()
            if elapsed_time > self.work_timeout:
                rospy.logwarn(f"AGV Controller: Arm work timeout after {elapsed_time:.1f}s - Resuming navigation")
                self.work_requested = False  # 작업 요청 플래그 리셋
                self.move_base_cancelled = False  # move_base 상태 리셋
                self.current_state = AGVState.IDLE
                self.work_start_time = None
                self._publish_status("Arm work timeout - Resuming navigation")
                return
        
        # 액션 서버 상태 로깅 (디버깅용)
        if state != actionlib.GoalStatus.PENDING and state != actionlib.GoalStatus.ACTIVE:
            rospy.loginfo(f"AGV Controller: Action server state changed to {state}")
        else:
            # 주기적으로 상태 확인 (10초마다)
            if hasattr(self, 'last_status_check_time'):
                if (rospy.Time.now() - self.last_status_check_time).to_sec() > 10.0:
                    rospy.loginfo(f"AGV Controller: Still waiting for arm work completion (state: {state})")
                    self.last_status_check_time = rospy.Time.now()
            else:
                self.last_status_check_time = rospy.Time.now()
        
        # 액션 서버를 통한 상태 확인 (백업)
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("AGV Controller: Arm work completed via action server - Resuming navigation")
            self.work_completed = True
            self.work_requested = False  # 작업 요청 플래그 리셋
            self.move_base_cancelled = False  # move_base 상태 리셋
            self.current_state = AGVState.IDLE  # 다음 waypoint로 계속
            self.work_start_time = None
            # 완료된 마커 ID 추가
            if hasattr(self, 'current_marker_info'):
                self.completed_markers.add(self.current_marker_info.marker_id)
                rospy.loginfo(f"AGV Controller: Added marker {self.current_marker_info.marker_id} to completed list")
            self._publish_status("Arm work completed - Resuming navigation")
            
        # 액션 서버 결과 확인 (더 확실한 방법)
        elif state == actionlib.GoalStatus.ACTIVE:
            # 액션이 활성 상태인 경우, 결과를 직접 확인
            try:
                result = self.marker_approach_client.get_result()
                if result is not None:
                    rospy.loginfo("AGV Controller: Action completed - Checking result")
                    # 결과가 있으면 성공으로 간주
                    self.work_completed = True
                    self.work_requested = False
                    self.move_base_cancelled = False
                    self.current_state = AGVState.IDLE
                    self.work_start_time = None
                    if hasattr(self, 'current_marker_info'):
                        self.completed_markers.add(self.current_marker_info.marker_id)
                        rospy.loginfo(f"AGV Controller: Added marker {self.current_marker_info.marker_id} to completed list")
                    self._publish_status("Arm work completed - Resuming navigation")
            except Exception as e:
                rospy.logdebug(f"AGV Controller: Error checking action result: {e}")
            
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("AGV Controller: Arm work failed via action server - Resuming navigation anyway")
            self.work_requested = False  # 작업 요청 플래그 리셋
            self.move_base_cancelled = False  # move_base 상태 리셋
            self.current_state = AGVState.IDLE
            self.work_start_time = None
            self._publish_status("Arm work failed - Resuming navigation")
            
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn("AGV Controller: Arm work preempted via action server - Resuming navigation")
            self.work_requested = False  # 작업 요청 플래그 리셋
            self.move_base_cancelled = False  # move_base 상태 리셋
            self.current_state = AGVState.IDLE
            self.work_start_time = None
            self._publish_status("Arm work preempted - Resuming navigation")
            
        elif state in [actionlib.GoalStatus.LOST, actionlib.GoalStatus.REJECTED]:
            rospy.logwarn(f"AGV Controller: Arm work lost/rejected via action server (state: {state}) - Resuming navigation")
            self.work_requested = False  # 작업 요청 플래그 리셋
            self.move_base_cancelled = False  # move_base 상태 리셋
            self.current_state = AGVState.IDLE
            self.work_start_time = None
            self._publish_status("Arm work lost - Resuming navigation")

    def _return_home(self):
        """시작점으로 복귀"""
        goal = MoveBaseGoal()
        goal.target_pose = self.home_position
        goal.target_pose.header.stamp = rospy.Time.now()
        
        rospy.loginfo("AGV Controller: Returning to home position")
        self.move_base_client.send_goal(goal)
        self.current_state = AGVState.RETURNING
        self._publish_status("Returning to home position")

    def _check_return_status(self):
        """복귀 상태 확인"""
        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("AGV Controller: Mission completed - Returned home")
            self.current_state = AGVState.COMPLETED
            self._publish_status("Mission completed")
            
        elif self.move_base_client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("AGV Controller: Return home failed - Retrying")
            self._return_home()

    def proceed_to_next_task(self):
        """로봇팔 작업 완료 후 다음 작업 진행"""
        if self.current_state == AGVState.WORKING:
            rospy.loginfo("AGV Controller: Arm work completed via status topic - Resuming navigation")
            self.work_completed = True
            self.work_requested = False
            self.move_base_cancelled = False
            self.current_state = AGVState.IDLE
            self.work_start_time = None
            
            # 완료된 마커 ID 추가
            if hasattr(self, 'current_marker_info'):
                self.completed_markers.add(self.current_marker_info.marker_id)
                rospy.loginfo(f"AGV Controller: Added marker {self.current_marker_info.marker_id} to completed list")
            
            self._publish_status("Arm work completed - Resuming navigation")
        else:
            rospy.logwarn(f"AGV Controller: Received COMPLETED status but AGV is in {self.current_state.name} state - Ignoring")

    def handle_arm_error(self):
        """로봇팔 작업 실패 처리"""
        if self.current_state == AGVState.WORKING:
            rospy.logwarn("AGV Controller: Arm work failed via status topic - Resuming navigation anyway")
            self.work_requested = False
            self.move_base_cancelled = False
            self.current_state = AGVState.IDLE
            self.work_start_time = None
            self._publish_status("Arm work failed - Resuming navigation")
        else:
            rospy.logwarn(f"AGV Controller: Received FAILED status but AGV is in {self.current_state.name} state - Ignoring")

    def wait_for_arm_completion(self):
        """로봇팔 작업 중 대기"""
        if self.current_state == AGVState.WORKING:
            rospy.logdebug("AGV Controller: Arm is working - Waiting for completion")
            # 작업 시작 시간이 설정되지 않은 경우 현재 시간으로 설정
            if self.work_start_time is None:
                self.work_start_time = rospy.Time.now()
        else:
            rospy.logdebug(f"AGV Controller: Received WORKING status but AGV is in {self.current_state.name} state - Ignoring")

    def _publish_status(self, status_msg):
        """상태 메시지 발행"""
        msg = String()
        msg.data = f"[{self.current_state.value}] {status_msg}"
        self.agv_status_pub.publish(msg)
        rospy.loginfo(f"AGV Status: {msg.data}")

    def shutdown(self):
        """노드 종료 처리"""
        rospy.loginfo("AGV Controller: Shutting down...")
        self.move_base_client.cancel_all_goals()
        self.marker_approach_client.cancel_all_goals()
        
        # AGV 정지
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        controller = AGVController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("AGV Controller: Node interrupted")
    except Exception as e:
        rospy.logerr(f"AGV Controller: Error - {e}")
