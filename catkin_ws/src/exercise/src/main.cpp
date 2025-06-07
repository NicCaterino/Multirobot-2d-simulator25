#include <ros/ros.h>
#include <memory>
#include <vector>
#include <map>
#include <opencv2/highgui.hpp>

#include "multirobot_simulator/world.h"
#include "multirobot_simulator/robot_ros.h"
#include "multirobot_simulator/lidar_ros.h"
#include "multirobot_simulator/config_parser.h"

class MultiRobotSimulator {
public:
    MultiRobotSimulator() : nh_("~") {
        // Parametri
        nh_.param<double>("simulation_frequency", sim_frequency_, 50.0);
        nh_.param<bool>("show_visualization", show_viz_, true);
        nh_.param<std::string>("map_directory", map_directory_, "../map/");
        
        // Timer per simulazione
        sim_timer_ = nh_.createTimer(ros::Duration(1.0 / sim_frequency_), 
                                    &MultiRobotSimulator::simulationStep, this);
        
        ROS_INFO("MultiRobotSimulator initialized at %.1f Hz", sim_frequency_);
    }
    
    ~MultiRobotSimulator() {
        if (show_viz_) {
            cv::destroyAllWindows();
        }
    }
    
    bool loadConfiguration(const std::string& config_file) {
        ConfigParser parser;
        if (!parser.loadFromFile(config_file)) {
            ROS_ERROR("Failed to load configuration from: %s", config_file.c_str());
            return false;
        }
        
        config_ = parser.getConfig();
        parser.printConfig();
        
        return initializeSimulation();
    }
    
private:
    ros::NodeHandle nh_;
    ros::Timer sim_timer_;
    
    // Parametri
    double sim_frequency_;
    bool show_viz_;
    std::string map_directory_;
    
    // Simulazione
    std::shared_ptr<World> world_;
    std::vector<std::shared_ptr<RobotROS>> robots_;
    std::vector<std::shared_ptr<LidarROS>> lidars_;
    std::map<int, std::shared_ptr<WorldItem>> items_by_id_;
    
    SimulationConfig config_;
    
    bool initializeSimulation() {
        // Crea il mondo
        world_ = std::make_shared<World>();
        
        // Carica la mappa
        std::string map_path = map_directory_ + config_.map_file;
        try {
            world_->loadFromImage(map_path);
            ROS_INFO("Loaded map: %s (%dx%d)", map_path.c_str(), world_->rows, world_->cols);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load map %s: %s", map_path.c_str(), e.what());
            return false;
        }
        
        // Crea i robot
        for (const auto& robot_config : config_.robots) {
            auto robot = createRobot(robot_config);
            if (robot) {
                robots_.push_back(robot);
                items_by_id_[robot_config.id] = robot;
                ROS_INFO("Created robot %d: %s", robot_config.id, robot_config.namespace_.c_str());
            }
        }
        
        // Crea i lidar
        for (const auto& lidar_config : config_.lidars) {
            auto lidar = createLidar(lidar_config);
            if (lidar) {
                lidars_.push_back(lidar);
                items_by_id_[lidar_config.id] = lidar;
                ROS_INFO("Created lidar %d: %s", lidar_config.id, lidar_config.namespace_.c_str());
            }
        }
        
        ROS_INFO("Simulation initialized with %zu robots and %zu lidars", 
                 robots_.size(), lidars_.size());
        
        return true;
    }
    
    std::shared_ptr<RobotROS> createRobot(const RobotConfig& config) {
        std::shared_ptr<RobotROS> robot;
        
        if (config.parent == -1) {
            // Robot senza parent (attaccato al mondo)
            robot = std::make_shared<RobotROS>(
                config.radius, world_, config.namespace_, config.frame_id,
                config.max_tv, config.max_rv, config.pose
            );
        } else {
            // Robot con parent (caso raro, ma supportato)
            auto parent_it = items_by_id_.find(config.parent);
            if (parent_it != items_by_id_.end()) {
                robot = std::make_shared<RobotROS>(
                    config.radius, parent_it->second, config.namespace_, config.frame_id,
                    config.max_tv, config.max_rv, config.pose
                );
            } else {
                ROS_ERROR("Robot %d: parent %d not found", config.id, config.parent);
                return nullptr;
            }
        }
        
        return robot;
    }
    
    std::shared_ptr<LidarROS> createLidar(const LidarConfig& config) {
        std::shared_ptr<LidarROS> lidar;
        
        if (config.parent == -1) {
            // Lidar senza parent (attaccato al mondo)
            lidar = std::make_shared<LidarROS>(
                config.fov, config.max_range, config.num_beams, world_,
                config.namespace_, config.frame_id, config.pose
            );
        } else {
            // Lidar con parent (tipicamente attaccato a un robot)
            auto parent_it = items_by_id_.find(config.parent);
            if (parent_it != items_by_id_.end()) {
                lidar = std::make_shared<LidarROS>(
                    config.fov, config.max_range, config.num_beams, parent_it->second,
                    config.namespace_, config.frame_id, config.pose
                );
            } else {
                ROS_ERROR("Lidar %d: parent %d not found", config.id, config.parent);
                return nullptr;
            }
        }
        
        return lidar;
    }
    
    void simulationStep(const ros::TimerEvent& event) {
        if (!world_) return;
        
        // Calcola dt
        static ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;
        
        // Limita dt per evitare salti temporali grandi
        dt = std::min(dt, 0.1);
        
        // Aggiorna la simulazione
        world_->timeTick(dt);
        
        // Visualizzazione (se abilitata)
        if (show_viz_) {
            world_->draw();
            char key = cv::waitKey(1);
            if (key == 27 || key == 'q') {  // ESC o 'q' per uscire
                ROS_INFO("Exiting simulation...");
                ros::shutdown();
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multirobot_simulator_node");
    
    // Verifica argomenti
    if (argc < 2) {
        ROS_ERROR("Usage: %s <config_file.json>", argv[0]);
        ROS_ERROR("Example: %s ../config/cappero_1r.json", argv[0]);
        return 1;
    }
    
    std::string config_file = argv[1];
    ROS_INFO("Loading configuration from: %s", config_file.c_str());
    
    // Crea il simulatore
    MultiRobotSimulator simulator;
    
    // Carica la configurazione e inizializza
    if (!simulator.loadConfiguration(config_file)) {
        ROS_ERROR("Failed to initialize simulation");
        return 1;
    }
    
    ROS_INFO("Multirobot simulation started. Press 'q' or ESC in the visualization window to exit.");
    
    // Avvia il loop ROS
    ros::spin();
    
    ROS_INFO("Simulation terminated.");
    return 0;
}