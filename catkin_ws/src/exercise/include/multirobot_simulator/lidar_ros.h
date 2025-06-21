#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include "multirobot_simulator/lidar.h"

// wrapper ROS per la classe Lidar base
class LidarROS : public Lidar {
public:
    // costruttore principale
    LidarROS(float fov_,
             float max_range_,
             int num_beams_,
             std::shared_ptr<World> w,
             const std::string& namespace_,
             const std::string& frame_id_,
             const Pose& pose_ = Pose::Identity());

    // costruttore con parent (di solito un robot)
    LidarROS(float fov_,
             float max_range_,
             int num_beams_,
             std::shared_ptr<WorldItem> p_,
             const std::string& namespace_,
             const std::string& frame_id_,
             const Pose& pose_ = Pose::Identity());
    
    ~LidarROS();
    
    // override del metodo base per aggiungere pubblicazione ROS
    void timeTick(float dt) override;
    
    // metodi per pubblicare su ROS
    void publishLaserScan();
    void publishTransform();
    
private:
    // componenti ROS
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // configurazione lidar
    std::string namespace_;
    std::string frame_id_;
    
    // timing per pubblicazione
    ros::Time last_scan_time_;
};