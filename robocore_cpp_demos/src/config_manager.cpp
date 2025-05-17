/**
 * @file config_manager.cpp
 * @brief Advanced parameter management implementation using an object-oriented design
 * 
 * This file implements a modern C++ parameter handling system with observer pattern
 * for robot configuration settings.
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>

#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <algorithm>

// Forward declarations
class ParameterManager;
class ConfigObserver;

/**
 * @brief Configuration parameter observer interface
 */
class ConfigObserver {
public:
    virtual ~ConfigObserver() = default;
    virtual void onParameterChanged(const std::string& name, const rclcpp::Parameter& param) = 0;
};

/**
 * @brief Central parameter management class
 */
class ParameterManager {
public:
    using ParameterList = std::vector<rclcpp::Parameter>;
    using ObserverPtr = std::shared_ptr<ConfigObserver>;
    
    explicit ParameterManager(rclcpp::Node::SharedPtr node);
    ~ParameterManager() = default;
    
    // Delete copy/move operations
    ParameterManager(const ParameterManager&) = delete;
    ParameterManager& operator=(const ParameterManager&) = delete;
    ParameterManager(ParameterManager&&) = delete;
    ParameterManager& operator=(ParameterManager&&) = delete;
    
    // Observer pattern methods
    void addObserver(ObserverPtr observer);
    void removeObserver(ObserverPtr observer);
    
    // Parameter access methods
    int getVelocity() const { return node_->get_parameter("velocity").as_int(); }
    std::string getLabel() const { return node_->get_parameter("label").as_string(); }
    bool getActive() const { return node_->get_parameter("active").as_bool(); }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::vector<ObserverPtr> observers_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    
    void initializeParameters();
    rcl_interfaces::msg::SetParametersResult validateParameters(const ParameterList& parameters);
    void notifyObservers(const rclcpp::Parameter& param);
    
    // Parameter constraint constants
    static constexpr int MIN_VELOCITY = 0;
    static constexpr int MAX_VELOCITY = 100;
    static constexpr int DEFAULT_VELOCITY = 75;
    static constexpr const char* DEFAULT_LABEL = "RoboCore";
    static constexpr bool DEFAULT_ACTIVE = true;
};

/**
 * @brief Concrete observer that logs parameter changes
 */
class LoggingObserver : public ConfigObserver {
public:
    explicit LoggingObserver(rclcpp::Logger logger) : logger_(logger) {}
    
    void onParameterChanged(const std::string& name, const rclcpp::Parameter& param) override {
        switch (param.get_type()) {
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                RCLCPP_INFO(logger_, "Parameter '%s' changed to %d", name.c_str(), param.as_int());
                break;
            case rclcpp::ParameterType::PARAMETER_STRING:
                RCLCPP_INFO(logger_, "Parameter '%s' changed to '%s'", name.c_str(), param.as_string().c_str());
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
                RCLCPP_INFO(logger_, "Parameter '%s' changed to %s", name.c_str(), param.as_bool() ? "true" : "false");
                break;
            default:
                RCLCPP_INFO(logger_, "Parameter '%s' changed (unhandled type)", name.c_str());
                break;
        }
    }
    
private:
    rclcpp::Logger logger_;
};

// Implementation of ParameterManager

ParameterManager::ParameterManager(rclcpp::Node::SharedPtr node) : node_(node) {
    initializeParameters();
    
    // Set up parameter validation callback
    using std::placeholders::_1;
    callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ParameterManager::validateParameters, this, _1)
    );
}

void ParameterManager::initializeParameters() {
    // Create velocity parameter descriptor with range constraints
    rcl_interfaces::msg::ParameterDescriptor velocity_desc;
    velocity_desc.name = "velocity";
    velocity_desc.description = "Robot velocity setting";
    velocity_desc.additional_constraints = "Value must be between 0 and 100";
    
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = MIN_VELOCITY;
    range.to_value = MAX_VELOCITY;
    range.step = 1;
    velocity_desc.integer_range.push_back(range);
    
    // Create label parameter descriptor
    rcl_interfaces::msg::ParameterDescriptor label_desc;
    label_desc.name = "label";
    label_desc.description = "Robot identification label";
    
    // Create active parameter descriptor
    rcl_interfaces::msg::ParameterDescriptor active_desc;
    active_desc.name = "active";
    active_desc.description = "Robot activation state";
    
    // Declare all parameters
    node_->declare_parameter("velocity", DEFAULT_VELOCITY, velocity_desc);
    node_->declare_parameter("label", DEFAULT_LABEL, label_desc);
    node_->declare_parameter("active", DEFAULT_ACTIVE, active_desc);
    
    RCLCPP_INFO(node_->get_logger(), "Configuration parameters initialized");
}

rcl_interfaces::msg::SetParametersResult ParameterManager::validateParameters(const ParameterList& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "velocity") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                result.successful = false;
                result.reason = "velocity must be an integer";
                break;
            }
            
            const int value = param.as_int();
            if (value < MIN_VELOCITY || value > MAX_VELOCITY) {
                result.successful = false;
                result.reason = "velocity must be between 0 and 100";
                break;
            }
            
            // Notify observers if validation passed
            if (result.successful) {
                notifyObservers(param);
            }
        }
        else if (param.get_name() == "label") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                result.successful = false;
                result.reason = "label must be a string";
                break;
            }
            
            const std::string value = param.as_string();
            if (value.empty()) {
                result.successful = false;
                result.reason = "label cannot be empty";
                break;
            }
            
            // Notify observers if validation passed
            if (result.successful) {
                notifyObservers(param);
            }
        }
        else if (param.get_name() == "active") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = "active must be a boolean";
                break;
            }
            
            // Notify observers if validation passed
            if (result.successful) {
                notifyObservers(param);
            }
        }
    }
    
    return result;
}

void ParameterManager::addObserver(ObserverPtr observer) {
    observers_.push_back(observer);
}

void ParameterManager::removeObserver(ObserverPtr observer) {
    observers_.erase(
        std::remove(observers_.begin(), observers_.end(), observer),
        observers_.end()
    );
}

void ParameterManager::notifyObservers(const rclcpp::Parameter& param) {
    for (const auto& observer : observers_) {
        observer->onParameterChanged(param.get_name(), param);
    }
}

/**
 * @brief Main entry point for the node
 */
int main(int argc, char* argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        // Create node
        auto node = std::make_shared<rclcpp::Node>("config_manager_node");
        
        // Create parameter manager
        auto param_manager = std::make_shared<ParameterManager>(node);
        
        // Add logging observer
        auto logger = std::make_shared<LoggingObserver>(node->get_logger());
        param_manager->addObserver(logger);
        
        RCLCPP_INFO(node->get_logger(), "Configuration manager initialized with defaults:");
        RCLCPP_INFO(node->get_logger(), "- Velocity: %d", param_manager->getVelocity());
        RCLCPP_INFO(node->get_logger(), "- Label: %s", param_manager->getLabel().c_str());
        RCLCPP_INFO(node->get_logger(), "- Active: %s", param_manager->getActive() ? "true" : "false");
        
        // Process events
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    // Clean up and exit
    rclcpp::shutdown();
    return 0;
}