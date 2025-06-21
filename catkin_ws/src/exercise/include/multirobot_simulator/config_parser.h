#pragma once

#include <string>
#include <vector>
#include <memory>
#include <json/json.h>

#include "multirobot_simulator/types.h"

// struttura per configurazione robot
struct RobotConfig {
    int id;
    std::string type;
    std::string frame_id;
    std::string namespace_;
    float radius;
    float max_rv;  // velocità angolare massima
    float max_tv;  // velocità lineare massima
    Pose pose;     // posizione iniziale
    int parent;    // ID del parent (-1 se nessuno)
};

// struttura per configurazione lidar
struct LidarConfig {
    int id;
    std::string type;
    std::string frame_id;
    std::string namespace_;
    float fov;         // campo visivo
    float max_range;   // range massimo
    int num_beams;     // numero di raggi
    Pose pose;         // posizione relativa al parent
    int parent;        // ID del parent (-1 se nessuno)
};

// configurazione completa della simulazione
struct SimulationConfig {
    std::string map_file;               // file PNG della mappa
    std::vector<RobotConfig> robots;    // lista robot
    std::vector<LidarConfig> lidars;    // lista lidar
};

// parser per file JSON di configurazione
class ConfigParser {
public:
    ConfigParser();
    ~ConfigParser();
    
    // carica configurazione da file JSON
    bool loadFromFile(const std::string& filename);
    
    // getter per la configurazione parsata
    const SimulationConfig& getConfig() const { return config_; }
    
    // stampa config per debug
    void printConfig() const;
    
private:
    SimulationConfig config_;
    
    // metodi helper per parsing
    Pose parsePose(const Json::Value& pose_array);
    bool parseRobot(const Json::Value& item, RobotConfig& robot);
    bool parseLidar(const Json::Value& item, LidarConfig& lidar);
};