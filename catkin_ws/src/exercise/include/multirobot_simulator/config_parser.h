#pragma once

#include <string>
#include <vector>
#include <memory>
#include <json/json.h>

#include "multirobot_simulator/types.h"

// Strutture per rappresentare la configurazione
struct RobotConfig {
    int id;
    std::string type;
    std::string frame_id;
    std::string namespace_;
    float radius;
    float max_rv;
    float max_tv;
    Pose pose;
    int parent;
};

struct LidarConfig {
    int id;
    std::string type;
    std::string frame_id;
    std::string namespace_;
    float fov;
    float max_range;
    int num_beams;
    Pose pose;
    int parent;
};

struct SimulationConfig {
    std::string map_file;
    std::vector<RobotConfig> robots;
    std::vector<LidarConfig> lidars;
};

class ConfigParser {
public:
    ConfigParser();
    ~ConfigParser();
    
    // Carica configurazione da file JSON
    bool loadFromFile(const std::string& filename);
    
    // Ottieni la configurazione parsata
    const SimulationConfig& getConfig() const { return config_; }
    
    // Utility per debug
    void printConfig() const;
    
private:
    SimulationConfig config_;
    
    // Helper per parsing
    Pose parsePose(const Json::Value& pose_array);
    bool parseRobot(const Json::Value& item, RobotConfig& robot);
    bool parseLidar(const Json::Value& item, LidarConfig& lidar);
};