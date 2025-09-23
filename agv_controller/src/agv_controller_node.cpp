#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <system_msgs/SystemState.h>
#include <system_msgs/NavigationState.h>
#include <system_msgs/MarkerInfo.h>

class AGVController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers
    ros::Publisher cmd_vel_pub_;
    ros::Publisher navigation_state_pub_;
    ros::Publisher cancel_goal_pub_;
    
    // Subscribers  
    ros::Subscriber system_state_sub_;
    ros::Subscriber marker_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber move_base_status_sub_;
    
    // Timers
    ros::Timer status_timer_;
    ros::Timer deceleration_timer_;
    
    // State variables
    std::string current_status_;
    bool is_decelerating_;
    bool is_stopped_;
    double current_speed_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose saved_goal_pose_;
    
    // Deceleration parameters
    std::vector<double> deceleration_steps_;
    int current_deceleration_step_;
    ros::Time deceleration_start_time_;
    double step_duration_;
    
    // Parameters
    double max_linear_speed_;
    double emergency_stop_decel_;
    bool navigation_active_;

public:
    AGVController() : 
        private_nh_("~"),
        current_status_("NAVIGATING"),
        is_decelerating_(false),
        is_stopped_(false),
        current_speed_(0.0),
        current_deceleration_step_(0),
        step_duration_(1.0),
        max_linear_speed_(0.5),
        emergency_stop_decel_(2.0),
        navigation_active_(false) {
        
        initializeParameters();
        initializePublishers();
        initializeSubscribers();
        initializeTimers();
        
        ROS_INFO("AGV Controller Node initialized");
    }

private:
    void initializeParameters() {
        // 파라미터 로드
        private_nh_.param<double>("max_linear_speed", max_linear_speed_, 0.5);
        private_nh_.param<double>("emergency_stop_decel", emergency_stop_decel_, 2.0);
        private_nh_.param<double>("step_duration", step_duration_, 1.0);
        
        // 감속 단계 설정
        deceleration_steps_ = {0.3, 0.2, 0.1, 0.05, 0.0};
        
        ROS_INFO("Parameters loaded - Max speed: %.2f m/s", max_linear_speed_);
    }
    
    void initializePublishers() {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        navigation_state_pub_ = nh_.advertise<system_msgs::NavigationState>("/navigation_state", 1);
        cancel_goal_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
        
        ROS_INFO("Publishers initialized");
    }
    
    void initializeSubscribers() {
        // 로봇팔에서 오는 시스템 상태
        system_state_sub_ = nh_.subscribe("/system_state", 1, 
                                        &AGVController::systemStateCallback, this);
        
        // 로봇팔에서 오는 마커 정보 (직접 수신도 가능)
        marker_info_sub_ = nh_.subscribe("/marker_detected", 1,
                                       &AGVController::markerInfoCallback, this);
        
        // AGV 자체 상태 정보
        odom_sub_ = nh_.subscribe("/odom", 1, &AGVController::odomCallback, this);
        move_base_status_sub_ = nh_.subscribe("/move_base/status", 1,
                                            &AGVController::moveBaseStatusCallback, this);
        
        ROS_INFO("Subscribers initialized");
    }
    
    void initializeTimers() {
        // 주기적 상태 발행 (2Hz)
        status_timer_ = nh_.createTimer(ros::Duration(0.5), 
                                      &AGVController::publishNavigationState, this);
        
        // 감속 제어 타이머 (10Hz)
        deceleration_timer_ = nh_.createTimer(ros::Duration(0.1),
                                            &AGVController::decelerationControl, this);
        
        ROS_INFO("Timers initialized");
    }
    
    // ===========================================
    // 콜백 함수들
    // ===========================================
    
    void systemStateCallback(const system_msgs::SystemState::ConstPtr& msg) {
        ROS_INFO("Received system state: %s, AGV state: %s", 
                 msg->system_state.c_str(), msg->agv_state.c_str());
        
        // 로봇팔의 시스템 상태에 따른 동작
        if (msg->system_state == "MARKER_DETECTED" && !is_decelerating_ && !is_stopped_) {
            ROS_INFO("Marker detected - starting deceleration sequence");
            startDeceleration();
        }
        else if (msg->system_state == "PICKUP_COMPLETED" && is_stopped_) {
            ROS_INFO("Pickup completed - resuming navigation");
            resumeNavigation();
        }
    }
    
    void markerInfoCallback(const system_msgs::MarkerInfo::ConstPtr& msg) {
        // 마커 직접 수신 시 처리 (선택사항)
        if (!is_decelerating_ && !is_stopped_) {
            ROS_INFO("Direct marker detection (ID: %d) - starting deceleration", msg->marker_id);
            startDeceleration();
        }
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 현재 위치 및 속도 업데이트
        current_pose_ = msg->pose.pose;
        current_speed_ = sqrt(pow(msg->twist.twist.linear.x, 2) + 
                            pow(msg->twist.twist.linear.y, 2));
    }
    
    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
        // move_base 상태 모니터링
        navigation_active_ = false;
        for (const auto& status : msg->status_list) {
            if (status.status == actionlib_msgs::GoalStatus::ACTIVE ||
                status.status == actionlib_msgs::GoalStatus::PENDING) {
                navigation_active_ = true;
                break;
            }
        }
    }
    
    // ===========================================
    // 감속 제어
    // ===========================================
    
    void startDeceleration() {
        if (is_decelerating_ || is_stopped_) {
            return;
        }
        
        is_decelerating_ = true;
        current_deceleration_step_ = 0;
        deceleration_start_time_ = ros::Time::now();
        current_status_ = "DECELERATING";
        
        // move_base 목표 취소 (부드러운 정차를 위해)
        cancelNavigation();
        
        ROS_INFO("Starting deceleration sequence");
    }
    
    void decelerationControl(const ros::TimerEvent& event) {
        if (!is_decelerating_) {
            return;
        }
        
        ros::Duration elapsed = ros::Time::now() - deceleration_start_time_;
        double expected_time = current_deceleration_step_ * step_duration_;
        
        // 다음 감속 단계로 넘어갈 시간인지 확인
        if (elapsed.toSec() >= expected_time && 
            current_deceleration_step_ < deceleration_steps_.size()) {
            
            double target_speed = deceleration_steps_[current_deceleration_step_];
            publishVelocityCommand(target_speed, 0.0);
            
            ROS_INFO("Deceleration step %d: target speed %.2f m/s", 
                     current_deceleration_step_, target_speed);
            
            current_deceleration_step_++;
            
            // 완전 정차 확인
            if (target_speed == 0.0) {
                completeDeceleration();
            }
        }
    }
    
    void completeDeceleration() {
        is_decelerating_ = false;
        is_stopped_ = true;
        current_status_ = "STOPPED";
        current_deceleration_step_ = 0;
        
        // 확실한 정지를 위해 한 번 더 0 속도 명령
        publishVelocityCommand(0.0, 0.0);
        
        ROS_INFO("Deceleration completed - AGV stopped");
    }
    
    void resumeNavigation() {
        if (!is_stopped_) {
            return;
        }
        
        is_stopped_ = false;
        is_decelerating_ = false;
        current_status_ = "RESUMING";
        
        // TODO: 원래 목표로 네비게이션 재개
        // 실제로는 saved_goal_pose_를 move_base에 재전송해야 함
        
        ros::Duration(2.0).sleep(); // 잠시 대기
        current_status_ = "NAVIGATING";
        
        ROS_INFO("Navigation resumed");
    }
    
    void emergencyStop() {
        ROS_WARN("Emergency stop activated");
        
        is_decelerating_ = false;
        is_stopped_ = true;
        current_status_ = "EMERGENCY_STOPPED";
        
        // 즉시 정지
        publishVelocityCommand(0.0, 0.0);
        cancelNavigation();
    }
    
    // ===========================================
    // 제어 명령
    // ===========================================
    
    void publishVelocityCommand(double linear_x, double angular_z) {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = angular_z;
        
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    void cancelNavigation() {
        actionlib_msgs::GoalID cancel_msg;
        cancel_goal_pub_.publish(cancel_msg);
        ROS_INFO("Navigation goal cancelled");
    }
    
    // ===========================================
    // 상태 발행
    // ===========================================
    
    void publishNavigationState(const ros::TimerEvent& event) {
        system_msgs::NavigationState nav_state;
        
        nav_state.status = current_status_;
        nav_state.current_pose = current_pose_;
        nav_state.current_speed = current_speed_;
        nav_state.timestamp = ros::Time::now();
        
        // TODO: 추가 정보 설정
        nav_state.distance_to_goal = 0.0;  // 실제 계산 필요
        nav_state.path_blocked = false;
        nav_state.goal_reached = (current_status_ == "STOPPED");
        
        navigation_state_pub_.publish(nav_state);
    }

public:
    void run() {
        ROS_INFO("AGV Controller running - waiting for commands from arm system");
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "agv_controller_node");
    
    try {
        AGVController controller;
        controller.run();
    } catch (const std::exception& e) {
        ROS_ERROR("AGV Controller error: %s", e.what());
        return -1;
    }
    
    return 0;
}