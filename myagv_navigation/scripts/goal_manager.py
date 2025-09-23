#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def reset_amcl_pose(x, y, yaw=0.0):
    """AMCL 초기 위치 재설정"""
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.pose.position.x = x
    pose_msg.pose.pose.position.y = y
    
    quat = quaternion_from_euler(0, 0, yaw)
    pose_msg.pose.pose.orientation.x = quat[0]
    pose_msg.pose.pose.orientation.y = quat[1]
    pose_msg.pose.pose.orientation.z = quat[2]
    pose_msg.pose.pose.orientation.w = quat[3]
    
    # 공분산 설정
    covariance = [0.0] * 36
    covariance[0] = 0.25
    covariance[7] = 0.25
    covariance[35] = 0.25
    pose_msg.pose.covariance = covariance
    
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    pub.publish(pose_msg)
    rospy.loginfo(f"AMCL 위치 재설정: ({x:.2f}, {y:.2f})")
    rospy.sleep(1.0)  # AMCL 안정화 대기

def send_goal(x, y, yaw=0.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    rospy.loginfo(f"목표 전송: x={x:.2f}, y={y:.2f}")
    
    success = client.wait_for_result(rospy.Duration(60.0))
    
    if success:
        state = client.get_state()
        if state == 3:  # SUCCEEDED
            rospy.loginfo("목표 도달 완료!")
            return True
        else:
            rospy.logwarn(f"목표 도달 실패 (상태: {state})")
            return False
    else:
        rospy.logwarn("목표 타임아웃")
        return False

if __name__ == "__main__":
    rospy.init_node("goal_manager")

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("move_base 서버 대기중...")
    client.wait_for_server()
    rospy.loginfo("move_base 서버 연결 완료")

    try:
        # 목적지1
        success1 = send_goal(1.20, 0.0)
        if success1:
            # AMCL 위치 재설정 (첫 번째 목적지 위치로)
            reset_amcl_pose(1.20, 0.0, 0.0)
            
            # 목적지2
            success2 = send_goal(1.73, 0.00,0.0)
            if success2:
                rospy.loginfo("모든 경로 완료")
            else:
                rospy.logerr("목적지2 실패")
        else:
            rospy.logerr("목적지1 실패")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("프로그램 중단됨")
