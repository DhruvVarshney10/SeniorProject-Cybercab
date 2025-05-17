/**
 * @file state_estimator.cpp
 * @brief Advanced Kalman filter implementation for robot state estimation
 * 
 * This implementation uses template metaprogramming and modern C++ features
 * to create a flexible, efficient state estimation system.
 */

#include "robocore_positioning/state_estimator.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <algorithm>
#include <functional>
#include <chrono>
#include <string>

// Forward declarations
template<typename StateType, typename MeasurementType>
class BayesianFilter;

template<typename StateType, typename MeasurementType>
class KalmanFilterImpl;

/**
 * @brief Base class for Bayesian filters
 */
template<typename StateType, typename MeasurementType>
class BayesianFilter {
public:
    virtual ~BayesianFilter() = default;
    
    // Core filter methods
    virtual void predict(const StateType& control_input) = 0;
    virtual void update(const MeasurementType& measurement) = 0;
    
    // Getters for filter state
    virtual StateType getState() const = 0;
    virtual double getUncertainty() const = 0;
};

/**
 * @brief Kalman filter implementation for scalar state
 */
template<typename StateType, typename MeasurementType>
class KalmanFilterImpl : public BayesianFilter<StateType, MeasurementType> {
public:
    KalmanFilterImpl(StateType initial_state, double initial_variance,
                  double process_noise, double measurement_noise)
        : state_(initial_state),
          variance_(initial_variance),
          process_noise_(process_noise),
          measurement_noise_(measurement_noise) {}

    void predict(const StateType& control_input) override {
        // Update state
        state_ += control_input;
        
        // Update variance
        variance_ += process_noise_;
    }
    
    void update(const MeasurementType& measurement) override {
        // Calculate Kalman gain
        double kalman_gain = variance_ / (variance_ + measurement_noise_);
        
        // Update state
        state_ = state_ + kalman_gain * (measurement - state_);
        
        // Update variance
        variance_ = (1.0 - kalman_gain) * variance_;
    }
    
    StateType getState() const override {
        return state_;
    }
    
    double getUncertainty() const override {
        return variance_;
    }
    
private:
    StateType state_;          // Estimated state
    double variance_;          // State uncertainty
    double process_noise_;     // Process noise variance
    double measurement_noise_; // Measurement noise variance
};

/**
 * @brief Node implementation for robot state estimation
 */
class StateEstimator : public rclcpp::Node {
public:
    explicit StateEstimator(const std::string& name) 
        : Node(name),
          is_first_odom_(true),
          last_angular_z_(0.0),
          imu_angular_z_(0.0) {
        
        // Declare and load parameters
        declare_parameter("initial_variance", 1000.0);
        declare_parameter("process_noise", 4.0);
        declare_parameter("measurement_noise", 0.5);
        
        double initial_variance = get_parameter("initial_variance").as_double();
        double process_noise = get_parameter("process_noise").as_double();
        double measurement_noise = get_parameter("measurement_noise").as_double();
        
        // Initialize Kalman filter
        filter_ = std::make_unique<KalmanFilterImpl<double, double>>(
            0.0, initial_variance, process_noise, measurement_noise);
        
        // Setup publishers and subscribers
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "robocore_control/odom_noisy", 10, 
            std::bind(&StateEstimator::processOdometry, this, std::placeholders::_1));
            
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/out", 1000, 
            std::bind(&StateEstimator::processImuData, this, std::placeholders::_1));
            
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "robocore_control/odom_kalman", 10);
            
        RCLCPP_INFO(get_logger(), "State estimator initialized");
        RCLCPP_INFO(get_logger(), "Parameter values:");
        RCLCPP_INFO(get_logger(), "  - Initial variance: %.2f", initial_variance);
        RCLCPP_INFO(get_logger(), "  - Process noise: %.2f", process_noise);
        RCLCPP_INFO(get_logger(), "  - Measurement noise: %.2f", measurement_noise);
    }
    
private:
    // State variables
    bool is_first_odom_;
    double last_angular_z_;
    double imu_angular_z_;
    nav_msgs::msg::Odometry filtered_odom_;
    
    // Filter
    std::unique_ptr<BayesianFilter<double, double>> filter_;
    
    // ROS communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    /**
     * @brief Process odometry data and run filter prediction step
     */
    void processOdometry(const nav_msgs::msg::Odometry& odom) {
        // Store the complete odometry message for later publishing
        filtered_odom_ = odom;
        
        // Initialize filter with first measurement if needed
        if (is_first_odom_) {
            last_angular_z_ = odom.twist.twist.angular.z;
            filter_ = std::make_unique<KalmanFilterImpl<double, double>>(
                odom.twist.twist.angular.z,
                get_parameter("initial_variance").as_double(),
                get_parameter("process_noise").as_double(),
                get_parameter("measurement_noise").as_double());
                
            is_first_odom_ = false;
            return;
        }
        
        // Calculate control input (motion model)
        double control_input = odom.twist.twist.angular.z - last_angular_z_;
        
        // Run prediction step
        filter_->predict(control_input);
        
        // Run update step with IMU measurement
        filter_->update(imu_angular_z_);
        
        // Update for next iteration
        last_angular_z_ = odom.twist.twist.angular.z;
        
        // Publish filtered odometry
        publishFilteredOdometry();
    }
    
    /**
     * @brief Process IMU data
     */
    void processImuData(const sensor_msgs::msg::Imu& imu) {
        // Store angular velocity for use in the filter
        imu_angular_z_ = imu.angular_velocity.z;
    }
    
    /**
     * @brief Publish filtered odometry data
     */
    void publishFilteredOdometry() {
        // Update angular velocity with filtered value
        filtered_odom_.twist.twist.angular.z = filter_->getState();
        
        // Add covariance information
        size_t angular_z_idx = 5; // Index for angular velocity around z in the covariance matrix
        filtered_odom_.twist.covariance[angular_z_idx * 6 + angular_z_idx] = filter_->getUncertainty();
        
        // Publish the message
        odom_pub_->publish(filtered_odom_);
        
        // Provide debug output occasionally
        static auto last_debug = this->now();
        auto current_time = this->now();
        if ((current_time - last_debug).seconds() > 5.0) {
            RCLCPP_DEBUG(get_logger(), 
                "Filter state: %.3f, Uncertainty: %.3f", 
                filter_->getState(), filter_->getUncertainty());
            last_debug = current_time;
        }
    }
};

/**
 * @brief Main entry point
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateEstimator>("state_estimator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}