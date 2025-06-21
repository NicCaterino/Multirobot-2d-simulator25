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

    // setup publisher per i dati laser
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

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/" + namespace_ + "/base_scan", 10);
    last_scan_time_ = ros::Time::now();

    ROS_INFO("LidarROS initialized: namespace=%s, frame_id=%s, beams=%d", 
             namespace_.c_str(), frame_id_.c_str(), num_beams);
}

LidarROS::~LidarROS() {
    ROS_INFO("LidarROS destroyed: %s", namespace_.c_str());
}

void LidarROS::timeTick(float dt) {
    // aggiorna i dati del lidar
    Lidar::timeTick(dt);
    
    // pubblica tutto su ROS
    publishLaserScan();
    publishTransform();
}

void LidarROS::publishLaserScan() {
    ros::Time scan_time = ros::Time::now();

    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = scan_time;
    scan_msg.header.frame_id = frame_id_;

    // parametri del laser
    scan_msg.angle_min = -fov / 2.0;
    scan_msg.angle_max = fov / 2.0;
    scan_msg.angle_increment = fov / num_beams;
    scan_msg.time_increment = 0.0;  // scan istantaneo
    scan_msg.scan_time = 0.1;       // frequenza circa 10Hz
    scan_msg.range_min = 0.1;
    scan_msg.range_max = max_range;

    // copia i dati delle distanze
    scan_msg.ranges.resize(num_beams);
    scan_msg.intensities.resize(num_beams);

    for (int i = 0; i < num_beams; ++i) {
        // se la distanza è valida la uso, altrimenti infinito
        if (ranges[i] >= scan_msg.range_min && ranges[i] <= scan_msg.range_max) {
            scan_msg.ranges[i] = ranges[i];
        } else {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        }
        
        scan_msg.intensities[i] = 100.0; // valore fisso per ora
    }

    scan_pub_.publish(scan_msg);
    last_scan_time_ = scan_time;
}

void LidarROS::publishTransform() {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.child_frame_id = frame_id_;

    if (parent) {
        // se ho un parent (robot) uso il frame relativo
        std::string parent_frame = namespace_ + "_base_link";
        transform_stamped.header.frame_id = parent_frame;

        Pose pose_to_use = pose_in_parent;
        transform_stamped.transform.translation.x = pose_to_use.translation().x();
        transform_stamped.transform.translation.y = pose_to_use.translation().y();
        transform_stamped.transform.translation.z = 0.0;

        float theta = Eigen::Rotation2Df(pose_to_use.linear()).angle();
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transform_stamped.transform.rotation = tf2::toMsg(q);
        
        ROS_DEBUG("Published transform %s -> %s", parent_frame.c_str(), frame_id_.c_str());
    } else {
        // altrimenti collegato direttamente a map
        transform_stamped.header.frame_id = "map";

        Pose world_pose = poseInWorld();
        transform_stamped.transform.translation.x = world_pose.translation().x();
        transform_stamped.transform.translation.y = world_pose.translation().y();
        transform_stamped.transform.translation.z = 0.0;

        float theta = Eigen::Rotation2Df(world_pose.linear()).angle();
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transform_stamped.transform.rotation = tf2::toMsg(q);
        
        ROS_DEBUG("Published transform map -> %s", frame_id_.c_str());
    }

    tf_broadcaster_.sendTransform(transform_stamped);
}