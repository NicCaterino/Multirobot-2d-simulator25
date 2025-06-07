#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include "multirobot_simulator/lidar.h"

class LidarROS : public Lidar {
public:
    LidarROS(float fov_, 
             float max_range_, 
             int num_beams_, 
             std::shared_ptr<World> w,
             const std::string& namespace_,
             const std::string& frame_id_,
             const Pose& pose_ = Pose::Identity());

    LidarROS(float fov_, 
             float max_range_, 
             int num_beams_, 
             std::shared_ptr<WorldItem> p_,
             const std::string& namespace_,
             const std::string& frame_id_,
             const Pose& pose_ = Pose::Identity());
    
    ~LidarROS();
    
    // Override del metodo base
    void timeTick(float dt) override;
    
    // Metodi ROS specifici
    void publishLaserScan();
    void publishTransform();
    
private:
    // ROS components
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Lidar configuration
    std::string namespace_;
    std::string frame_id_;
    
    // Timing
    ros::Time last_scan_time_;
};