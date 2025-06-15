#include "multirobot_simulator/robot_ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/// Costruttore principale: inizializza un robot ROS con tutto quello che serve (odom, cmd_vel, ecc.)
RobotROS::RobotROS(float radius_, 
                   std::shared_ptr<World> w_, 
                   const std::string& namespace_,
                   const std::string& frame_id_,
                   float max_tv_,
                   float max_rv_,
                   const Pose& pose_)
    : Robot(radius_, w_, pose_), 
      namespace_(namespace_),
      frame_id_(frame_id_),
      max_tv_(max_tv_),
      max_rv_(max_rv_),
      first_update_(true) {

    odom_frame_id_ = namespace_ + "/odom";

    // Pubblica l'odometria e ascolta i comandi di movimento
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10);
    cmd_vel_sub_ = nh_.subscribe("/" + namespace_ + "/cmd_vel", 1, 
                                &RobotROS::cmdVelCallback, this);
    
    last_time_ = ros::Time::now();
    last_pose_ = pose_in_parent;

    ROS_INFO("RobotROS initialized: namespace=%s, frame_id=%s", 
             namespace_.c_str(), frame_id_.c_str());
}

/// Costruttore alternativo se il robot ha un altro parent (tipo un altro oggetto del mondo)
RobotROS::RobotROS(float radius_, 
                   std::shared_ptr<WorldItem> parent_,
                   const std::string& namespace_,
                   const std::string& frame_id_,
                   float max_tv_,
                   float max_rv_,
                   const Pose& pose_)
    : Robot(radius_, parent_, pose_), 
      namespace_(namespace_),
      frame_id_(frame_id_),
      max_tv_(max_tv_),
      max_rv_(max_rv_),
      first_update_(true) {

    odom_frame_id_ = namespace_ + "/odom";

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10);
    cmd_vel_sub_ = nh_.subscribe("/" + namespace_ + "/cmd_vel", 1, 
                                &RobotROS::cmdVelCallback, this);

    last_time_ = ros::Time::now();
    last_pose_ = pose_in_parent;

    ROS_INFO("RobotROS initialized: namespace=%s, frame_id=%s", 
             namespace_.c_str(), frame_id_.c_str());
}

/// Distruttore, stampa solo che il robot è stato distrutto (giusto per pulizia)
RobotROS::~RobotROS() {
    ROS_INFO("RobotROS destroyed: %s", namespace_.c_str());
}

/// Riceve comandi di velocità da ROS e li salva, limitando quelli troppo alti
void RobotROS::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    tv = std::max(-max_tv_, std::min(max_tv_, static_cast<float>(msg->linear.x)));
    rv = std::max(-max_rv_, std::min(max_rv_, static_cast<float>(msg->angular.z)));
    current_twist_ = *msg; // salvo il messaggio intero anche se uso solo x e z
}

/// Chiamata ad ogni "tick" del simulatore: muove il robot e pubblica odometria/transform
void RobotROS::timeTick(float dt) {
    Robot::timeTick(dt); // muove il robot nella simulazione
    publishOdometry();   // pubblica la posizione
    publishTransform();  // pubblica anche la trasformazione tf
}

/// Crea e pubblica un messaggio di odometria in base alla posizione e velocità attuale
void RobotROS::publishOdometry() {
    ros::Time current_time = ros::Time::now();

    Pose world_pose = poseInWorld();

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = frame_id_;

    odom_msg.pose.pose.position.x = world_pose.translation().x();
    odom_msg.pose.pose.position.y = world_pose.translation().y();
    odom_msg.pose.pose.position.z = 0.0;

    float theta = Eigen::Rotation2Df(world_pose.linear()).angle();
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = tv;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = rv;

    // Covarianze base
    for (int i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.01;
    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_pub_.publish(odom_msg);
    last_time_ = current_time;
    last_pose_ = world_pose;
}

/// Manda su tf la trasformazione attuale del robot (da "map" a frame del robot)
void RobotROS::publishTransform() {
    Pose world_pose = poseInWorld();

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = frame_id_;

    transform_stamped.transform.translation.x = world_pose.translation().x();
    transform_stamped.transform.translation.y = world_pose.translation().y();
    transform_stamped.transform.translation.z = 0.0;

    float theta = Eigen::Rotation2Df(world_pose.linear()).angle();
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transform_stamped.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_.sendTransform(transform_stamped);
    ROS_DEBUG("Published transform map -> %s", transform_stamped.child_frame_id.c_str());
}
