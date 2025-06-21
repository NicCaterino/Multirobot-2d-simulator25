#include "multirobot_simulator/config_parser.h"
#include <fstream>
#include <iostream>
#include <ros/ros.h>

ConfigParser::ConfigParser() {
    // costruttore vuoto, tanto non c'è niente da inizializzare
}

ConfigParser::~ConfigParser() {
    // distruttore
}

bool ConfigParser::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Non riesco ad aprire il file: %s", filename.c_str());
        return false;
    }
    
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    
    // parsing del JSON - se fallisce restituisce errore
    if (!Json::parseFromStream(builder, file, &root, &errors)) {
        ROS_ERROR("Errore nel parsing JSON: %s", errors.c_str());
        return false;
    }
    
    // controllo campo mappa
    if (!root.isMember("map") || !root["map"].isString()) {
        ROS_ERROR("Campo 'map' mancante o non valido");
        return false;
    }
    config_.map_file = root["map"].asString();
    
    // controllo array items
    if (!root.isMember("items") || !root["items"].isArray()) {
        ROS_ERROR("Array 'items' mancante o non valido");
        return false;
    }
    
    const Json::Value& items = root["items"];
    
    // pulisco la config precedente (se c'era)
    config_.robots.clear();
    config_.lidars.clear();
    
    // ciclo su tutti gli items
    for (const Json::Value& item : items) {
        if (!item.isMember("type") || !item["type"].isString()) {
            ROS_WARN("Item senza campo 'type', lo salto");
            continue;
        }
        
        std::string type = item["type"].asString();
        
        if (type == "robot") {
            RobotConfig robot;
            if (parseRobot(item, robot)) {
                config_.robots.push_back(robot);
                ROS_INFO("Caricato robot: id=%d, namespace=%s", robot.id, robot.namespace_.c_str());
            } else {
                ROS_WARN("Fallito parsing robot");
            }
        } else if (type == "lidar") {
            LidarConfig lidar;
            if (parseLidar(item, lidar)) {
                config_.lidars.push_back(lidar);
                ROS_INFO("Caricato lidar: id=%d, namespace=%s", lidar.id, lidar.namespace_.c_str());
            } else {
                ROS_WARN("Fallito parsing lidar");
            }
        } else {
            ROS_WARN("Tipo sconosciuto: %s", type.c_str());
        }
    }
    
    ROS_INFO("Config caricata: %zu robot, %zu lidar", 
             config_.robots.size(), config_.lidars.size());
    
    return true;
}

Pose ConfigParser::parsePose(const Json::Value& pose_array) {
    Pose pose = Pose::Identity();
    
    // controllo che sia un array di 3 elementi
    if (!pose_array.isArray() || pose_array.size() != 3) {
        ROS_WARN("Array pose non valido, uso identità");
        return pose;
    }
    
    // estraggo x, y, theta
    float x = pose_array[0].asFloat();
    float y = pose_array[1].asFloat();
    float theta = pose_array[2].asFloat();
    
    pose.translation() << x, y;
    pose.rotate(theta);
    
    return pose;
}

bool ConfigParser::parseRobot(const Json::Value& item, RobotConfig& robot) {
    // controllo tutti i campi obbligatori
    
    if (!item.isMember("id") || !item["id"].isInt()) {
        ROS_ERROR("Robot senza campo 'id'");
        return false;
    }
    robot.id = item["id"].asInt();
    
    if (!item.isMember("frame_id") || !item["frame_id"].isString()) {
        ROS_ERROR("Robot senza 'frame_id'");
        return false;
    }
    robot.frame_id = item["frame_id"].asString();
    
    if (!item.isMember("namespace") || !item["namespace"].isString()) {
        ROS_ERROR("Robot senza 'namespace'");
        return false;
    }
    robot.namespace_ = item["namespace"].asString();
    
    if (!item.isMember("radius") || !item["radius"].isNumeric()) {
        ROS_ERROR("Robot senza 'radius'");
        return false;
    }
    robot.radius = item["radius"].asFloat();
    
    if (!item.isMember("max_rv") || !item["max_rv"].isNumeric()) {
        ROS_ERROR("Robot senza 'max_rv'");
        return false;
    }
    robot.max_rv = item["max_rv"].asFloat();
    
    if (!item.isMember("max_tv") || !item["max_tv"].isNumeric()) {
        ROS_ERROR("Robot senza 'max_tv'");
        return false;
    }
    robot.max_tv = item["max_tv"].asFloat();
    
    if (!item.isMember("pose") || !item["pose"].isArray()) {
        ROS_ERROR("Robot senza 'pose'");
        return false;
    }
    robot.pose = parsePose(item["pose"]);
    
    if (!item.isMember("parent") || !item["parent"].isInt()) {
        ROS_ERROR("Robot senza 'parent'");
        return false;
    }
    robot.parent = item["parent"].asInt();
    
    robot.type = "robot";
    
    return true;
}

bool ConfigParser::parseLidar(const Json::Value& item, LidarConfig& lidar) {
    // come sui robot
    
    if (!item.isMember("id") || !item["id"].isInt()) {
        ROS_ERROR("Lidar senza 'id'");
        return false;
    }
    lidar.id = item["id"].asInt();
    
    if (!item.isMember("frame_id") || !item["frame_id"].isString()) {
        ROS_ERROR("Lidar senza 'frame_id'");
        return false;
    }
    lidar.frame_id = item["frame_id"].asString();
    
    if (!item.isMember("namespace") || !item["namespace"].isString()) {
        ROS_ERROR("Lidar senza 'namespace'");
        return false;
    }
    lidar.namespace_ = item["namespace"].asString();
    
    if (!item.isMember("fov") || !item["fov"].isNumeric()) {
        ROS_ERROR("Lidar senza 'fov'");
        return false;
    }
    lidar.fov = item["fov"].asFloat();
    
    if (!item.isMember("max_range") || !item["max_range"].isNumeric()) {
        ROS_ERROR("Lidar senza 'max_range'");
        return false;
    }
    lidar.max_range = item["max_range"].asFloat();
    
    if (!item.isMember("num_beams") || !item["num_beams"].isInt()) {
        ROS_ERROR("Lidar senza 'num_beams'");
        return false;
    }
    lidar.num_beams = item["num_beams"].asInt();
    
    if (!item.isMember("pose") || !item["pose"].isArray()) {
        ROS_ERROR("Lidar senza 'pose'");
        return false;
    }
    lidar.pose = parsePose(item["pose"]);
    
    if (!item.isMember("parent") || !item["parent"].isInt()) {
        ROS_ERROR("Lidar senza 'parent'");
        return false;
    }
    lidar.parent = item["parent"].asInt();
    
    lidar.type = "lidar";
    
    return true;
}

void ConfigParser::printConfig() const {
    ROS_INFO("=== Config Simulazione ===");
    ROS_INFO("File mappa: %s", config_.map_file.c_str());
    
    ROS_INFO("Robot (%zu):", config_.robots.size());
    for (const auto& robot : config_.robots) {
        ROS_INFO("  Robot %d: ns=%s, frame=%s, raggio=%.2f, parent=%d", 
                 robot.id, robot.namespace_.c_str(), robot.frame_id.c_str(), 
                 robot.radius, robot.parent);
    }
    
    ROS_INFO("Lidar (%zu):", config_.lidars.size());
    for (const auto& lidar : config_.lidars) {
        ROS_INFO("  Lidar %d: ns=%s, frame=%s, raggi=%d, parent=%d", 
                 lidar.id, lidar.namespace_.c_str(), lidar.frame_id.c_str(), 
                 lidar.num_beams, lidar.parent);
    }
    ROS_INFO("=== Fine Config ===");
}