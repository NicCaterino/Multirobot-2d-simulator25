#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "multirobot_simulator/robot.h"

class RobotROS : public Robot {
public:
    RobotROS(float radius_, 
             std::shared_ptr<World> w_, 
             const std::string& namespace_,
             const std::string& frame_id_,
             float max_tv_ = 1.0,
             float max_rv_ = 1.0,
             const Pose& pose_ = Pose::Identity());
    
    RobotROS(float radius_, 
             std::shared_ptr<WorldItem> parent_,
             const std::string& namespace_,
             const std::string& frame_id_,
             float max_tv_ = 1.0,
             float max_rv_ = 1.0,
             const Pose& pose_ = Pose::Identity());
    
    ~RobotROS();
    
    // Override dei metodi base
    void timeTick(float dt) override;
    
    // Metodi ROS specifici
    void publishOdometry();
    void publishTransform();
    
private:
    // Callback per cmd_vel
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // ROS components
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Robot configuration
    std::string namespace_;
    std::string frame_id_;
    std::string odom_frame_id_;
    float max_tv_, max_rv_;
    
    // Odometry tracking
    ros::Time last_time_;
    bool first_update_;
    
    // Per calcolare velocità da odometria
    Pose last_pose_;
    geometry_msgs::Twist current_twist_;
};