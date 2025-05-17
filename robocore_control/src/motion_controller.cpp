/**
 * @file motion_controller.cpp
 * @brief Advanced robot motion control implementation using strategy pattern
 * 
 * This implementation separates robot kinematics from the controller logic using
 * the strategy pattern for different control algorithms.
 */

#include "robocore_control/motion_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <functional>
#include <algorithm>
#include <mutex>

// Forward declarations
class KinematicsStrategy;
class OdometryCalculator;
class TransformPublisher;

/**
 * @brief Base class for kinematics strategy implementations
 */
class KinematicsStrategy {
public:
    virtual ~KinematicsStrategy() = default;
    
    // Forward kinematics: converts robot velocities to wheel velocities
    virtual Eigen::Vector2d calculateWheelVelocities(double linear_velocity, double angular_velocity) const = 0;
    
    // Inverse kinematics: converts wheel positions to robot position
    virtual void updateRobotPose(double left_wheel_pos, double right_wheel_pos,
                                double dt, double& x, double& y, double& theta) const = 0;
};

/**
 * @brief Differential drive kinematics implementation
 */
class DifferentialDriveKinematics : public KinematicsStrategy {
public:
    DifferentialDriveKinematics(double wheel_radius, double wheel_separation)
        : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation) {
        
        // Precompute conversion matrix for efficiency
        speed_conversion_ << wheel_radius_/2, wheel_radius_/2, 
                            wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    }
    
    Eigen::Vector2d calculateWheelVelocities(double linear_velocity, double angular_velocity) const override {
        Eigen::Vector2d robot_speed(linear_velocity, angular_velocity);
        return speed_conversion_.inverse() * robot_speed;
    }
    
    void updateRobotPose(double left_wheel_pos, double right_wheel_pos,
                        double dt, double& x, double& y, double& theta) const override {
        
        // Calculate the positional change
        double d_left = left_wheel_pos * wheel_radius_;
        double d_right = right_wheel_pos * wheel_radius_;
        
        // Calculate the position increment
        double d_s = (d_right + d_left) / 2.0;
        double d_theta = (d_right - d_left) / wheel_separation_;
        
        // Update pose
        theta += d_theta;
        x += d_s * cos(theta);
        y += d_s * sin(theta);
    }
    
private:
    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
};

/**
 * @brief Handles odometry message creation and publishing
 */
class OdometryCalculator {
public:
    OdometryCalculator(rclcpp::Node* node, const std::string& odom_frame, const std::string& base_frame)
        : node_(node), last_update_time_(node->get_clock()->now()) {
        
        // Initialize odometry message
        odom_msg_.header.frame_id = odom_frame;
        odom_msg_.child_frame_id = base_frame;
        
        // Initialize publisher
        odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/robocore_control/odom", 10);
    }
    
    void update(double x, double y, double theta, double linear_vel, double angular_vel) {
        auto current_time = node_->get_clock()->now();
        
        // Update pose
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        
        odom_msg_.header.stamp = current_time;
        odom_msg_.pose.pose.position.x = x;
        odom_msg_.pose.pose.position.y = y;
        odom_msg_.pose.pose.position.z = 0.0;
        odom_msg_.pose.pose.orientation.x = q.getX();
        odom_msg_.pose.pose.orientation.y = q.getY();
        odom_msg_.pose.pose.orientation.z = q.getZ();
        odom_msg_.pose.pose.orientation.w = q.getW();
        
        // Update velocity
        odom_msg_.twist.twist.linear.x = linear_vel;
        odom_msg_.twist.twist.angular.z = angular_vel;
        
        // Publish odometry
        odom_publisher_->publish(odom_msg_);
        
        // Update time
        last_update_time_ = current_time;
    }
    
    rclcpp::Time getLastUpdateTime() const {
        return last_update_time_;
    }
    
private:
    rclcpp::Node* node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    nav_msgs::msg::Odometry odom_msg_;
    rclcpp::Time last_update_time_;
};

/**
 * @brief Handles transform broadcasting
 */
class TransformPublisher {
public:
    TransformPublisher(rclcpp::Node* node, const std::string& odom_frame, const std::string& base_frame)
        : node_(node) {
        
        // Initialize transform
        transform_stamped_.header.frame_id = odom_frame;
        transform_stamped_.child_frame_id = base_frame;
        
        // Initialize broadcaster
        transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    }
    
    void update(double x, double y, double theta) {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        
        transform_stamped_.transform.translation.x = x;
        transform_stamped_.transform.translation.y = y;
        transform_stamped_.transform.translation.z = 0.0;
        transform_stamped_.transform.rotation.x = q.getX();
        transform_stamped_.transform.rotation.y = q.getY();
        transform_stamped_.transform.rotation.z = q.getZ();
        transform_stamped_.transform.rotation.w = q.getW();
        
        transform_stamped_.header.stamp = node_->get_clock()->now();
        transform_broadcaster_->sendTransform(transform_stamped_);
    }
    
private:
    rclcpp::Node* node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

/**
 * @brief Advanced motion controller implementation
 */
class MotionController : public rclcpp::Node {
public:
    explicit MotionController(const std::string& name) 
        : Node(name),
          left_wheel_prev_pos_(0.0),
          right_wheel_prev_pos_(0.0),
          x_(0.0),
          y_(0.0),
          theta_(0.0) {
        
        // Load parameters
        declare_parameter("wheel_radius", 0.033);
        declare_parameter("wheel_separation", 0.17);
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_footprint");
        
        double wheel_radius = get_parameter("wheel_radius").as_double();
        double wheel_separation = get_parameter("wheel_separation").as_double();
        std::string odom_frame = get_parameter("odom_frame").as_string();
        std::string base_frame = get_parameter("base_frame").as_string();
        
        RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius: " << wheel_radius);
        RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation: " << wheel_separation);
        
        // Initialize kinematics strategy
        kinematics_ = std::make_unique<DifferentialDriveKinematics>(wheel_radius, wheel_separation);
        
        // Initialize odometry calculator
        odometry_ = std::make_unique<OdometryCalculator>(this, odom_frame, base_frame);
        
        // Initialize transform publisher
        transform_publisher_ = std::make_unique<TransformPublisher>(this, odom_frame, base_frame);
        
        // Create publishers and subscribers
        wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/simple_velocity_controller/commands", 10);
            
        vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/robocore_control/cmd_vel", 10, 
            std::bind(&MotionController::velocityCallback, this, std::placeholders::_1));
            
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MotionController::jointStateCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(get_logger(), "Motion controller initialized");
    }
    
private:
    // Robot state
    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;
    double x_;
    double y_;
    double theta_;
    std::mutex state_mutex_;
    
    // Strategy objects
    std::unique_ptr<KinematicsStrategy> kinematics_;
    std::unique_ptr<OdometryCalculator> odometry_;
    std::unique_ptr<TransformPublisher> transform_publisher_;
    
    // ROS communication
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    void velocityCallback(const geometry_msgs::msg::TwistStamped& msg) {
        // Get robot velocity commands
        double linear_velocity = msg.twist.linear.x;
        double angular_velocity = msg.twist.angular.z;
        
        // Convert to wheel velocities using kinematics strategy
        Eigen::Vector2d wheel_speeds = kinematics_->calculateWheelVelocities(linear_velocity, angular_velocity);
        
        // Prepare and publish wheel command message
        std_msgs::msg::Float64MultiArray wheel_cmd_msg;
        wheel_cmd_msg.data.push_back(wheel_speeds.coeff(1)); // Left wheel
        wheel_cmd_msg.data.push_back(wheel_speeds.coeff(0)); // Right wheel
        
        wheel_cmd_pub_->publish(wheel_cmd_msg);
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState& msg) {
        // Lock state for thread safety
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        try {
            // Extract wheel positions
            double left_wheel_pos = msg.position.at(1);
            double right_wheel_pos = msg.position.at(0);
            
            // Calculate position changes
            double dp_left = left_wheel_pos - left_wheel_prev_pos_;
            double dp_right = right_wheel_pos - right_wheel_prev_pos_;
            
            // Get current time and calculate time delta
            rclcpp::Time current_time = msg.header.stamp;
            double dt = (current_time - odometry_->getLastUpdateTime()).seconds();
            
            if (dt <= 0.0) {
                RCLCPP_WARN(get_logger(), "Invalid time delta: %f", dt);
                return;
            }
            
            // Save current positions for next iteration
            left_wheel_prev_pos_ = left_wheel_pos;
            right_wheel_prev_pos_ = right_wheel_pos;
            
            // Calculate wheel angular velocities
            double omega_left = dp_left / dt;
            double omega_right = dp_right / dt;
            
            // Update robot pose using kinematics strategy
            kinematics_->updateRobotPose(dp_left, dp_right, dt, x_, y_, theta_);
            
            // Calculate robot velocities
            // For simplicity, we'll reuse the kinematics calculation
            Eigen::Vector2d wheel_speeds(omega_right, omega_left);
            Eigen::Matrix2d conversion_matrix;
            conversion_matrix << 1.0, 1.0, 1.0, -1.0;
            Eigen::Vector2d robot_speeds = conversion_matrix * wheel_speeds * 0.5 * get_parameter("wheel_radius").as_double();
            
            double linear_velocity = robot_speeds.coeff(0);
            double angular_velocity = robot_speeds.coeff(1) / get_parameter("wheel_separation").as_double();
            
            // Update odometry and publish
            odometry_->update(x_, y_, theta_, linear_velocity, angular_velocity);
            
            // Update and publish transform
            transform_publisher_->update(x_, y_, theta_);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error processing joint state: %s", e.what());
        }
    }
};

/**
 * @brief Main entry point
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionController>("motion_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}