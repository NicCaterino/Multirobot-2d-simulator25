#include "multirobot_simulator/config_parser.h"
#include <fstream>
#include <iostream>
#include <ros/ros.h>

ConfigParser::ConfigParser() {
}

ConfigParser::~ConfigParser() {
}

bool ConfigParser::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open config file: %s", filename.c_str());
        return false;
    }
    
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    
    if (!Json::parseFromStream(builder, file, &root, &errors)) {
        ROS_ERROR("JSON parsing error: %s", errors.c_str());
        return false;
    }
    
    // Parse map file
    if (!root.isMember("map") || !root["map"].isString()) {
        ROS_ERROR("Missing or invalid 'map' field in config file");
        return false;
    }
    config_.map_file = root["map"].asString();
    
    // Parse items array
    if (!root.isMember("items") || !root["items"].isArray()) {
        ROS_ERROR("Missing or invalid 'items' array in config file");
        return false;
    }
    
    const Json::Value& items = root["items"];
    
    // Clear previous config
    config_.robots.clear();
    config_.lidars.clear();
    
    // Parse each item
    for (const Json::Value& item : items) {
        if (!item.isMember("type") || !item["type"].isString()) {
            ROS_WARN("Item missing 'type' field, skipping");
            continue;
        }
        
        std::string type = item["type"].asString();
        
        if (type == "robot") {
            RobotConfig robot;
            if (parseRobot(item, robot)) {
                config_.robots.push_back(robot);
                ROS_INFO("Loaded robot: id=%d, namespace=%s", robot.id, robot.namespace_.c_str());
            } else {
                ROS_WARN("Failed to parse robot configuration");
            }
        } else if (type == "lidar") {
            LidarConfig lidar;
            if (parseLidar(item, lidar)) {
                config_.lidars.push_back(lidar);
                ROS_INFO("Loaded lidar: id=%d, namespace=%s", lidar.id, lidar.namespace_.c_str());
            } else {
                ROS_WARN("Failed to parse lidar configuration");
            }
        } else {
            ROS_WARN("Unknown item type: %s", type.c_str());
        }
    }
    
    ROS_INFO("Configuration loaded: %zu robots, %zu lidars", 
             config_.robots.size(), config_.lidars.size());
    
    return true;
}

Pose ConfigParser::parsePose(const Json::Value& pose_array) {
    Pose pose = Pose::Identity();
    
    if (!pose_array.isArray() || pose_array.size() != 3) {
        ROS_WARN("Invalid pose array, using identity");
        return pose;
    }
    
    float x = pose_array[0].asFloat();
    float y = pose_array[1].asFloat();
    float theta = pose_array[2].asFloat();
    
    pose.translation() << x, y;
    pose.rotate(theta);
    
    return pose;
}

bool ConfigParser::parseRobot(const Json::Value& item, RobotConfig& robot) {
    // Required fields
    if (!item.isMember("id") || !item["id"].isInt()) {
        ROS_ERROR("Robot missing 'id' field");
        return false;
    }
    robot.id = item["id"].asInt();
    
    if (!item.isMember("frame_id") || !item["frame_id"].isString()) {
        ROS_ERROR("Robot missing 'frame_id' field");
        return false;
    }
    robot.frame_id = item["frame_id"].asString();
    
    if (!item.isMember("namespace") || !item["namespace"].isString()) {
        ROS_ERROR("Robot missing 'namespace' field");
        return false;
    }
    robot.namespace_ = item["namespace"].asString();
    
    if (!item.isMember("radius") || !item["radius"].isNumeric()) {
        ROS_ERROR("Robot missing 'radius' field");
        return false;
    }
    robot.radius = item["radius"].asFloat();
    
    if (!item.isMember("max_rv") || !item["max_rv"].isNumeric()) {
        ROS_ERROR("Robot missing 'max_rv' field");
        return false;
    }
    robot.max_rv = item["max_rv"].asFloat();
    
    if (!item.isMember("max_tv") || !item["max_tv"].isNumeric()) {
        ROS_ERROR("Robot missing 'max_tv' field");
        return false;
    }
    robot.max_tv = item["max_tv"].asFloat();
    
    if (!item.isMember("pose") || !item["pose"].isArray()) {
        ROS_ERROR("Robot missing 'pose' field");
        return false;
    }
    robot.pose = parsePose(item["pose"]);
    
    if (!item.isMember("parent") || !item["parent"].isInt()) {
        ROS_ERROR("Robot missing 'parent' field");
        return false;
    }
    robot.parent = item["parent"].asInt();
    
    robot.type = "robot";
    
    return true;
}

bool ConfigParser::parseLidar(const Json::Value& item, LidarConfig& lidar) {
    // Required fields
    if (!item.isMember("id") || !item["id"].isInt()) {
        ROS_ERROR("Lidar missing 'id' field");
        return false;
    }
    lidar.id = item["id"].asInt();
    
    if (!item.isMember("frame_id") || !item["frame_id"].isString()) {
        ROS_ERROR("Lidar missing 'frame_id' field");
        return false;
    }
    lidar.frame_id = item["frame_id"].asString();
    
    if (!item.isMember("namespace") || !item["namespace"].isString()) {
        ROS_ERROR("Lidar missing 'namespace' field");
        return false;
    }
    lidar.namespace_ = item["namespace"].asString();
    
    if (!item.isMember("fov") || !item["fov"].isNumeric()) {
        ROS_ERROR("Lidar missing 'fov' field");
        return false;
    }
    lidar.fov = item["fov"].asFloat();
    
    if (!item.isMember("max_range") || !item["max_range"].isNumeric()) {
        ROS_ERROR("Lidar missing 'max_range' field");
        return false;
    }
    lidar.max_range = item["max_range"].asFloat();
    
    if (!item.isMember("num_beams") || !item["num_beams"].isInt()) {
        ROS_ERROR("Lidar missing 'num_beams' field");
        return false;
    }
    lidar.num_beams = item["num_beams"].asInt();
    
    if (!item.isMember("pose") || !item["pose"].isArray()) {
        ROS_ERROR("Lidar missing 'pose' field");
        return false;
    }
    lidar.pose = parsePose(item["pose"]);
    
    if (!item.isMember("parent") || !item["parent"].isInt()) {
        ROS_ERROR("Lidar missing 'parent' field");
        return false;
    }
    lidar.parent = item["parent"].asInt();
    
    lidar.type = "lidar";
    
    return true;
}

void ConfigParser::printConfig() const {
    ROS_INFO("=== Simulation Configuration ===");
    ROS_INFO("Map file: %s", config_.map_file.c_str());
    
    ROS_INFO("Robots (%zu):", config_.robots.size());
    for (const auto& robot : config_.robots) {
        ROS_INFO("  Robot %d: namespace=%s, frame=%s, radius=%.2f, parent=%d", 
                 robot.id, robot.namespace_.c_str(), robot.frame_id.c_str(), 
                 robot.radius, robot.parent);
    }
    
    ROS_INFO("Lidars (%zu):", config_.lidars.size());
    for (const auto& lidar : config_.lidars) {
        ROS_INFO("  Lidar %d: namespace=%s, frame=%s, beams=%d, parent=%d", 
                 lidar.id, lidar.namespace_.c_str(), lidar.frame_id.c_str(), 
                 lidar.num_beams, lidar.parent);
    }
    ROS_INFO("=== End Configuration ===");
}