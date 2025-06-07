#include "multirobot_simulator/lidar_ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

LidarROS::LidarROS(float fov_, 
                   float max_range_, 
                   int num_beams_, 
                   std::shared_ptr<World> w,
                   const std::string& namespace_,
                   const std::string& frame_id_,
                   const Pose& pose_)
    : Lidar(fov_, max_range_, num_beams_, w, pose_),
      namespace_(namespace_),
      frame_id_(frame_id_) {
    
    // Setup ROS publisher
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/" + namespace_ + "/base_scan", 10);
    
    last_scan_time_ = ros::Time::now();
    
    ROS_INFO("LidarROS initialized: namespace=%s, frame_id=%s, beams=%d", 
             namespace_.c_str(), frame_id_.c_str(), num_beams);
}

LidarROS::LidarROS(float fov_, 
                   float max_range_, 
                   int num_beams_, 
                   std::shared_ptr<WorldItem> p_,
                   const std::string& namespace_,
                   const std::string& frame_id_,
                   const Pose& pose_)
    : Lidar(fov_, max_range_, num_beams_, p_, pose_),
      namespace_(namespace_),
      frame_id_(frame_id_) {
    
    // Setup ROS publisher
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/" + namespace_ + "/base_scan", 10);
    
    last_scan_time_ = ros::Time::now();
    
    ROS_INFO("LidarROS initialized: namespace=%s, frame_id=%s, beams=%d", 
             namespace_.c_str(), frame_id_.c_str(), num_beams);
}

LidarROS::~LidarROS() {
    ROS_INFO("LidarROS destroyed: %s", namespace_.c_str());
}

void LidarROS::timeTick(float dt) {
    // Chiama il metodo base per calcolare le distanze
    Lidar::timeTick(dt);
    
    // Pubblica i dati laser e transform
    publishLaserScan();
    publishTransform();
}

void LidarROS::publishLaserScan() {
    ros::Time scan_time = ros::Time::now();
    
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = scan_time;
    scan_msg.header.frame_id = frame_id_;
    
    // Parametri del laser
    scan_msg.angle_min = -fov / 2.0;
    scan_msg.angle_max = fov / 2.0;
    scan_msg.angle_increment = fov / num_beams;
    scan_msg.time_increment = 0.0;  // Assume scansione istantanea
    scan_msg.scan_time = 0.1;       // 10Hz di default
    scan_msg.range_min = 0.1;       // Distanza minima ragionevole
    scan_msg.range_max = max_range;
    
    // Copia i dati delle distanze
    scan_msg.ranges.resize(num_beams);
    scan_msg.intensities.resize(num_beams);
    
    for (int i = 0; i < num_beams; ++i) {
        // Se il range è valido, usa il valore calcolato
        if (ranges[i] >= scan_msg.range_min && ranges[i] <= scan_msg.range_max) {
            scan_msg.ranges[i] = ranges[i];
        } else {
            // Range non valido - usa infinito per indicare "nessun ostacolo"
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        }
        
        // Intensità fissa (potrebbe essere parametrizzata)
        scan_msg.intensities[i] = 100.0;
    }
    
    scan_pub_.publish(scan_msg);
    last_scan_time_ = scan_time;
}

void LidarROS::publishTransform() {
    // Pubblica transform da parent frame a lidar frame
    Pose world_pose = poseInWorld();
    
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    
    // Se ha un parent, pubblica transform relativo
    if (parent) {
        // Per semplicità, usiamo il frame del robot parent
        // In un sistema più complesso, dovremmo gestire la gerarchia completa
        transform_stamped.header.frame_id = "robot_base_link";  // Assumiamo questo nome
    } else {
        transform_stamped.header.frame_id = "map";
    }
    
    transform_stamped.child_frame_id = frame_id_;
    
    // Se ha parent, usa pose relativa, altrimenti usa pose mondiale
    Pose pose_to_use = parent ? pose_in_parent : world_pose;
    
    // Posizione
    transform_stamped.transform.translation.x = pose_to_use.translation().x();
    transform_stamped.transform.translation.y = pose_to_use.translation().y();
    transform_stamped.transform.translation.z = 0.0;
    
    // Orientamento
    float theta = Eigen::Rotation2Df(pose_to_use.linear()).angle();
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transform_stamped.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_.sendTransform(transform_stamped);
}