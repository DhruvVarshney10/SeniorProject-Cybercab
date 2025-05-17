/**
 * @file emergency_halt.cpp
 * @brief Safety controller implementation using a component-based design
 * 
 * This implementation uses a clean architecture with separation of concerns
 * between collision detection, visualization, and robot control.
 */

#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <math.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

/**
 * @brief Safety states enumeration
 */
enum class SafetyState {
    SAFE,       // No obstacles in range
    WARNING,    // Obstacle detected in warning zone
    DANGER      // Obstacle detected in danger zone
};

// Forward declarations
class CollisionDetector;
class ZoneVisualizer;
class SpeedController;

/**
 * @brief Collision detection component
 * 
 * Analyzes sensor data to detect potential collisions
 */
class CollisionDetector {
public:
    CollisionDetector(double warning_distance, double danger_distance)
        : warning_distance_(warning_distance),
          danger_distance_(danger_distance),
          current_state_(SafetyState::SAFE) {}
          
    SafetyState analyzeScan(const sensor_msgs::msg::LaserScan& scan) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Default to safe state
        SafetyState new_state = SafetyState::SAFE;
        
        // Find minimum distance in scan
        double min_distance = std::numeric_limits<double>::infinity();
        for (const auto& range : scan.ranges) {
            if (std::isfinite(range) && range < min_distance) {
                min_distance = range;
            }
        }
        
        // Determine state based on minimum distance
        if (min_distance <= danger_distance_) {
            new_state = SafetyState::DANGER;
        } else if (min_distance <= warning_distance_) {
            new_state = SafetyState::WARNING;
        }
        
        // Update current state
        current_state_ = new_state;
        return current_state_;
    }
    
    SafetyState getCurrentState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_state_;
    }
    
    double getWarningDistance() const { return warning_distance_; }
    double getDangerDistance() const { return danger_distance_; }
    
private:
    double warning_distance_;
    double danger_distance_;
    SafetyState current_state_;
    mutable std::mutex state_mutex_;
};

/**
 * @brief Zone visualization component
 * 
 * Creates visual markers for safety zones
 */
class ZoneVisualizer {
public:
    ZoneVisualizer(const std::shared_ptr<CollisionDetector>& detector)
        : detector_(detector), frame_id_("") {
        
        initializeMarkers();
    }
    
    visualization_msgs::msg::MarkerArray createVisualization(SafetyState state) {
        switch (state) {
            case SafetyState::DANGER:
                zones_.markers[0].color.a = 1.0; // Warning zone
                zones_.markers[1].color.a = 1.0; // Danger zone
                break;
                
            case SafetyState::WARNING:
                zones_.markers[0].color.a = 1.0; // Warning zone
                zones_.markers[1].color.a = 0.5; // Danger zone
                break;
                
            case SafetyState::SAFE:
                zones_.markers[0].color.a = 0.5; // Warning zone
                zones_.markers[1].color.a = 0.5; // Danger zone
                break;
        }
        
        return zones_;
    }
    
    void setFrameId(const std::string& frame_id) {
        frame_id_ = frame_id;
        for (auto& marker : zones_.markers) {
            marker.header.frame_id = frame_id_;
        }
    }
    
private:
    std::shared_ptr<CollisionDetector> detector_;
    visualization_msgs::msg::MarkerArray zones_;
    std::string frame_id_;
    
    void initializeMarkers() {
        // Create warning zone marker
        visualization_msgs::msg::Marker warning_zone;
        warning_zone.id = 0;
        warning_zone.type = visualization_msgs::msg::Marker::CYLINDER;
        warning_zone.action = visualization_msgs::msg::Marker::ADD;
        warning_zone.scale.z = 0.001;
        warning_zone.scale.x = detector_->getWarningDistance() * 2;
        warning_zone.scale.y = detector_->getWarningDistance() * 2;
        warning_zone.color.r = 1.0;
        warning_zone.color.g = 0.984;
        warning_zone.color.b = 0.0;
        warning_zone.color.a = 0.5;
        zones_.markers.push_back(warning_zone);
        
        // Create danger zone marker
        visualization_msgs::msg::Marker danger_zone = warning_zone;
        danger_zone.id = 1;
        danger_zone.scale.x = detector_->getDangerDistance() * 2;
        danger_zone.scale.y = detector_->getDangerDistance() * 2;
        danger_zone.color.r = 1.0;
        danger_zone.color.g = 0.0;
        danger_zone.color.b = 0.0;
        danger_zone.color.a = 0.5;
        danger_zone.pose.position.z = 0.001;
        zones_.markers.push_back(danger_zone);
    }
};

/**
 * @brief Speed controller component
 * 
 * Controls robot speed based on safety state
 */
class SpeedController {
public:
    using JoyTurboAction = twist_mux_msgs::action::JoyTurbo;
    using GoalHandleJoyTurbo = rclcpp_action::ClientGoalHandle<JoyTurboAction>;
    
    SpeedController(rclcpp::Node* node) 
        : node_(node),
          current_speed_level_(1),
          is_emergency_stop_(false) {
        
        // Create action clients
        decrease_speed_client_ = rclcpp_action::create_client<JoyTurboAction>(
            node_, "joy_turbo_decrease");
            
        increase_speed_client_ = rclcpp_action::create_client<JoyTurboAction>(
            node_, "joy_turbo_increase");
            
        // Create emergency halt publisher
        emergency_halt_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "emergency_halt", 10);
            
        // Wait for action servers
        waitForActionServers();
    }
    
    void handleStateChange(SafetyState old_state, SafetyState new_state) {
        if (old_state == new_state) {
            return;
        }
        
        switch (new_state) {
            case SafetyState::DANGER:
                setEmergencyStop(true);
                break;
                
            case SafetyState::WARNING:
                setEmergencyStop(false);
                decreaseSpeed();
                break;
                
            case SafetyState::SAFE:
                setEmergencyStop(false);
                increaseSpeed();
                break;
        }
    }
    
private:
    rclcpp::Node* node_;
    rclcpp_action::Client<JoyTurboAction>::SharedPtr decrease_speed_client_;
    rclcpp_action::Client<JoyTurboAction>::SharedPtr increase_speed_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_halt_pub_;
    int current_speed_level_;
    bool is_emergency_stop_;
    
    void waitForActionServers() {
        bool decrease_ready = false;
        bool increase_ready = false;
        
        // Try to connect to both action servers with timeout
        for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
            if (!decrease_ready) {
                decrease_ready = decrease_speed_client_->wait_for_action_server(1s);
                if (!decrease_ready) {
                    RCLCPP_WARN(node_->get_logger(), 
                        "Action /joy_turbo_decrease not available! Retry %d/5", i+1);
                }
            }
            
            if (!increase_ready) {
                increase_ready = increase_speed_client_->wait_for_action_server(1s);
                if (!increase_ready) {
                    RCLCPP_WARN(node_->get_logger(), 
                        "Action /joy_turbo_increase not available! Retry %d/5", i+1);
                }
            }
            
            if (decrease_ready && increase_ready) {
                RCLCPP_INFO(node_->get_logger(), "Connected to speed control action servers");
                return;
            }
            
            std::this_thread::sleep_for(2s);
        }
        
        if (!decrease_ready || !increase_ready) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to connect to speed control action servers. "
                "Emergency braking will still work, but speed control is disabled.");
        }
    }
    
    void setEmergencyStop(bool enable) {
        if (enable == is_emergency_stop_) {
            return;
        }
        
        is_emergency_stop_ = enable;
        std_msgs::msg::Bool msg;
        msg.data = enable;
        emergency_halt_pub_->publish(msg);
        
        if (enable) {
            RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP ACTIVATED");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Emergency stop deactivated");
        }
    }
    
    void decreaseSpeed() {
        if (current_speed_level_ <= 0) {
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Decreasing speed due to obstacle");
        sendSpeedChangeGoal(decrease_speed_client_);
        current_speed_level_--;
    }
    
    void increaseSpeed() {
        if (current_speed_level_ >= 2) {
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Increasing speed - path is clear");
        sendSpeedChangeGoal(increase_speed_client_);
        current_speed_level_++;
    }
    
    void sendSpeedChangeGoal(rclcpp_action::Client<JoyTurboAction>::SharedPtr client) {
        if (!client->action_server_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Speed control action server not available");
            return;
        }
        
        auto goal = JoyTurboAction::Goal();
        client->async_send_goal(goal);
    }
};

/**
 * @brief Main safety system node
 */
class SafetySystem : public rclcpp::Node {
public:
    SafetySystem() : Node("emergency_halt_node"), 
                     previous_state_(SafetyState::SAFE),
                     is_first_scan_(true) {
        
        // Declare and get parameters
        declare_parameter<double>("warning_distance", 0.6);
        declare_parameter<double>("danger_distance", 0.2);
        declare_parameter<std::string>("scan_topic", "scan");
        
        const double warning_distance = get_parameter("warning_distance").as_double();
        const double danger_distance = get_parameter("danger_distance").as_double();
        const std::string scan_topic = get_parameter("scan_topic").as_string();
        
        RCLCPP_INFO(get_logger(), "Safety system initializing");
        RCLCPP_INFO(get_logger(), "Warning distance: %.2f m", warning_distance);
        RCLCPP_INFO(get_logger(), "Danger distance: %.2f m", danger_distance);
        
        // Create components
        detector_ = std::make_shared<CollisionDetector>(warning_distance, danger_distance);
        visualizer_ = std::make_shared<ZoneVisualizer>(detector_);
        controller_ = std::make_shared<SpeedController>(this);
        
        // Create publishers and subscribers
        zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("safety_zones", 10);
        
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&SafetySystem::processScan, this, std::placeholders::_1));
            
        RCLCPP_INFO(get_logger(), "Safety system initialized");
    }
    
private:
    std::shared_ptr<CollisionDetector> detector_;
    std::shared_ptr<ZoneVisualizer> visualizer_;
    std::shared_ptr<SpeedController> controller_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;
    
    SafetyState previous_state_;
    bool is_first_scan_;
    
    void processScan(const sensor_msgs::msg::LaserScan& scan) {
        // Initialize frame ID on first scan
        if (is_first_scan_) {
            visualizer_->setFrameId(scan.header.frame_id);
            is_first_scan_ = false;
        }
        
        // Analyze scan data
        SafetyState current_state = detector_->analyzeScan(scan);
        
        // Handle state changes
        if (current_state != previous_state_) {
            controller_->handleStateChange(previous_state_, current_state);
            previous_state_ = current_state;
        }
        
        // Update visualization
        zones_pub_->publish(visualizer_->createVisualization(current_state));
    }
};

/**
 * @brief Main entry point
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetySystem>());
    rclcpp::shutdown();
    return 0;
}