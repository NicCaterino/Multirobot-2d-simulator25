#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "multirobot_simulator/robot.h"

// wrapper ROS per la classe Robot base
class RobotROS : public Robot {
public:
    // costruttore principale
    RobotROS(float radius_,
             std::shared_ptr<World> w_,
             const std::string& namespace_,
             const std::string& frame_id_,
             float max_tv_ = 1.0,
             float max_rv_ = 1.0,
             const Pose& pose_ = Pose::Identity());
    
    // costruttore con parent
    RobotROS(float radius_,
             std::shared_ptr<WorldItem> parent_,
             const std::string& namespace_,
             const std::string& frame_id_,
             float max_tv_ = 1.0,
             float max_rv_ = 1.0,
             const Pose& pose_ = Pose::Identity());
    
    ~RobotROS();
    
    // override del metodo base per aggiungere pubblicazione ROS
    void timeTick(float dt) override;
    
    // metodi per pubblicare su ROS
    void publishOdometry();
    void publishTransform();
    
private:
    // callback per ricevere comandi di velocità
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // componenti ROS
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // configurazione robot
    std::string namespace_;
    std::string frame_id_;
    std::string odom_frame_id_;
    float max_tv_, max_rv_;  // limiti di velocità
    
    // tracking per odometria
    ros::Time last_time_;
    bool first_update_;
    
    // stato precedente per calcoli
    Pose last_pose_;
    geometry_msgs::Twist current_twist_;
};