#include <iostream>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "segway_rmp_ros2/msg/segway_status_stamped.hpp"

#include "libsegwayrmp/segwayrmp.h"

#include <boost/thread.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2::toMsg
#include <chrono>

using namespace std::chrono_literals;  // For defining time durations in seconds/milliseconds easily.

// update rate of handle status callback
constexpr std::chrono::milliseconds updateInterval(50); // 50 and 100 seems to be solving trajectory problem.

class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;

static double radians_to_degrees = 180.0 / M_PI;
static double degrees_to_radians = M_PI / 180.0;

// Message Wrappers
void handleDebugMessages(const std::string &msg) {RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "%s", msg.c_str());}
void handleInfoMessages(const std::string &msg) {RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", msg.c_str());}
void handleErrorMessages(const std::string &msg) {RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", msg.c_str());}

void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss);

namespace segwayrmp {
    SegwayRMP::~SegwayRMP() {
        // Cleanup code here, if needed
    }
}

// ROS Node class
class SegwayRMPNode : public rclcpp::Node {
public:
    SegwayRMPNode() : Node("segway_rmp_node_ros2") {
        this->segway_rmp = nullptr;
        this->first_odometry = true;
        this->last_forward_displacement = 0.0;
        this->last_yaw_displacement = 0.0;
        this->odometry_x = 0.0;
        this->odometry_y = 0.0;
        this->odometry_w = 0.0;
        this->linear_vel = 0.0;
        this->angular_vel = 0.0;
        this->target_linear_vel = 0.0;
        this->target_angular_vel = 0.0;
        this->initial_integrated_forward_position = 0.0;
        this->initial_integrated_left_wheel_position = 0.0;
        this->initial_integrated_right_wheel_position = 0.0;
        this->initial_integrated_turn_position = 0.0;
        this->count = 0;
    }
    
    ~SegwayRMPNode() {
        this->disconnect();
    }
    
    void disconnect() {
        if (this->segway_rmp != nullptr)
            delete this->segway_rmp;
        this->segway_rmp = nullptr;
    }

    void run() {
        if (this->getParameters()) {
            return;
        }

        this->setupSegwayRMP();
        this->setupROSComms();

        // Setup keep-alive timer
        this->keep_alive_timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 20.0), 
            std::bind(&SegwayRMPNode::keepAliveCallback, this)
        );

        this->odometry_reset_start_time = this->now();
        this->connected = false;

        while (rclcpp::ok()) {
            try {
                this->segway_rmp->connect();
                this->connected = true;
            } catch (const std::exception &e) {
                std::string e_msg(e.what());
                RCLCPP_ERROR(this->get_logger(), "Exception while connecting to the SegwayRMP, check your cables and power buttons.");
                RCLCPP_ERROR(this->get_logger(), "    %s", e_msg.c_str());
                this->connected = false;
            }
            
            if (this->spin()) {  // ROS 2 is OK, but we aren't connected, wait then try again
                RCLCPP_WARN(this->get_logger(), "Not connected to the SegwayRMP, will retry in 5 seconds...");
                rclcpp::sleep_for(std::chrono::seconds(5));
            }           
        }
    }

    bool spin() {
        // rclcpp::Rate rate(20);  // past -->  50 Hz

        if (rclcpp::ok() && this->connected) {
            RCLCPP_INFO(this->get_logger(), "Segway RMP Ready.");
            while (rclcpp::ok() && this->connected) {
                rclcpp::spin_some(shared_from_this());
                // rate.sleep();
            }
        }
        
        if (rclcpp::ok()) {  // Error, but not shutdown
            return true;
        } else {  // Shutdown
            return false;
        }
        
        
    }

    //  Note: The velocities should be +/-
    // angular velocity = 28.647890
    // linear velocity = 0.750000
    void keepAliveCallback() {

        if (!this->connected || this->reset_odometry)
            return;

        if (rclcpp::ok()) {
            std::lock_guard<std::mutex> lock(this->m_mutex);

            // Update the linear velocity based on the linear acceleration limits
            if (this->linear_vel < this->target_linear_vel) {
                if (this->linear_pos_accel_limit == 0.0 
                    || this->target_linear_vel - this->linear_vel < this->linear_pos_accel_limit)
                    this->linear_vel = this->target_linear_vel;
                else
                    this->linear_vel += this->linear_pos_accel_limit; 
            } else if (this->linear_vel > this->target_linear_vel) {
                if (this->linear_neg_accel_limit == 0.0 
                    || this->linear_vel - this->target_linear_vel < this->linear_neg_accel_limit)
                    this->linear_vel = this->target_linear_vel;
                else
                    this->linear_vel -= this->linear_neg_accel_limit; 
            }

            // Update the angular velocity based on the angular acceleration limits
            if (this->angular_vel < this->target_angular_vel) {
                if (this->angular_pos_accel_limit == 0.0
                    || this->target_angular_vel - this->angular_vel < this->angular_pos_accel_limit)
                    this->angular_vel = this->target_angular_vel;
                else
                    this->angular_vel += this->angular_pos_accel_limit; 
            } else if (this->angular_vel > this->target_angular_vel) {
                if (this->angular_neg_accel_limit == 0.0 
                    || this->angular_vel - this->target_angular_vel < this->angular_neg_accel_limit)
                    this->angular_vel = this->target_angular_vel;
                else
                    this->angular_vel -= this->angular_neg_accel_limit; 
            }

            // std::cout << "linear: " << this->linear_vel << " <---> " << "Angular: " <<  this->angular_vel << std::endl;

            try {
                // Store the commanded and sent velocities for logging.
                this->sent_linear_vel = this->linear_vel;
                this->sent_angular_vel = this->angular_vel * degrees_to_radians; // Store in rad/s for consistency.

                // Log the move information.
                if (this->log_level == "debug")
                {
                    RCLCPP_INFO(this->get_logger(), "Move command received: linear velocity = %f, angular velocity = %f", 
                        this->linear_vel, this->angular_vel);
                    RCLCPP_INFO(this->get_logger(), "Velocities sent to Segway RMP: linear velocity = %f, angular velocity = %f", 
                        this->linear_vel, this->angular_vel);
                }
            
                // Call the move function with the calculated scale.
                this->segway_rmp->move(this->linear_vel, this->angular_vel);
            } catch (const std::exception &e) {
                std::string e_msg(e.what());
                RCLCPP_ERROR(this->get_logger(), "Error commanding Segway RMP: %s", e_msg.c_str());
                this->connected = false;
                this->disconnect();
            }
        }
    }

    void handleStatus(segwayrmp::SegwayStatus::Ptr &ss_ptr) {

        if (!this->connected)
            return;

        // handleStatus callback rate manager
        static auto lastUpdateTime = std::chrono::steady_clock::now();
        auto currentTime = std::chrono::steady_clock::now();
        if (currentTime - lastUpdateTime < updateInterval) {
            // Not enough time has passed; return immediately.
            return;
        }
        lastUpdateTime = currentTime;

        
        // Get the time
        rclcpp::Time current_time = this->now();

        this->sss_msg.header.stamp = current_time;
        
        segwayrmp::SegwayStatus &ss = *(ss_ptr);

        // Check if an odometry reset is still required
        if (this->reset_odometry) {
            if ((current_time - this->odometry_reset_start_time).seconds() < 0.25) {
                return; // discard readings for the first 0.25 seconds
            }
            if (fabs(ss.integrated_forward_position) < 1e-3 &&
                fabs(ss.integrated_turn_position) < 1e-3 &&
                fabs(ss.integrated_left_wheel_position) < 1e-3 &&
                fabs(ss.integrated_right_wheel_position) < 1e-3) {
                this->initial_integrated_forward_position = ss.integrated_forward_position;
                this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
                this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
                this->initial_integrated_turn_position = ss.integrated_turn_position;
                RCLCPP_INFO(this->get_logger(), "Integrators reset by Segway RMP successfully");
                this->reset_odometry = false;
            } else if ((current_time - this->odometry_reset_start_time).seconds() > this->odometry_reset_duration) {
                this->initial_integrated_forward_position = ss.integrated_forward_position;
                this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
                this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
                this->initial_integrated_turn_position = ss.integrated_turn_position;
                RCLCPP_INFO(this->get_logger(), "Integrator reset by Segway RMP failed. Performing software reset"); 
                this->reset_odometry = false;
            } else {
                return; // continue waiting for odometry to be reset
            }
        }

        // Fill in the status message
        this->sss_msg.segway.pitch_angle = ss.pitch * degrees_to_radians;
        this->sss_msg.segway.pitch_rate = ss.pitch_rate * degrees_to_radians;
        this->sss_msg.segway.roll_angle = ss.roll * degrees_to_radians;
        this->sss_msg.segway.roll_rate = ss.roll_rate * degrees_to_radians;
        this->sss_msg.segway.left_wheel_velocity = ss.left_wheel_speed;
        this->sss_msg.segway.right_wheel_velocity = ss.right_wheel_speed;
        this->sss_msg.segway.yaw_rate = ss.yaw_rate * degrees_to_radians;
        this->sss_msg.segway.servo_frames = ss.servo_frames;
        this->sss_msg.segway.left_wheel_displacement = 
            ss.integrated_left_wheel_position - this->initial_integrated_left_wheel_position;
        this->sss_msg.segway.right_wheel_displacement = 
            ss.integrated_right_wheel_position - this->initial_integrated_right_wheel_position;
        this->sss_msg.segway.forward_displacement = 
            ss.integrated_forward_position - this->initial_integrated_forward_position;
        this->sss_msg.segway.yaw_displacement = 
            (ss.integrated_turn_position - this->initial_integrated_turn_position) * degrees_to_radians;
        this->sss_msg.segway.left_motor_torque = ss.left_motor_torque;
        this->sss_msg.segway.right_motor_torque = ss.right_motor_torque;
        this->sss_msg.segway.operation_mode = ss.operational_mode;
        this->sss_msg.segway.gain_schedule = ss.controller_gain_schedule;
        this->sss_msg.segway.ui_battery = ss.ui_battery_voltage;
        this->sss_msg.segway.powerbase_battery = ss.powerbase_battery_voltage;
        this->sss_msg.segway.motors_enabled = static_cast<bool>(ss.motor_status);
        
        this->segway_status_pub->publish(this->sss_msg);
        
        // Odometry calculation and publishing
        float forward_displacement = 
            (ss.integrated_forward_position - this->initial_integrated_forward_position) * 
            this->linear_odom_scale;
        // delete
        // std::cout << "forward_displacement --> " << forward_displacement << std::endl;
        // std::cout << "this->initial_integrated_forward_position -->"<< this->initial_integrated_forward_position <<  std::endl;
        // std::cout << "(ss.integrated_forward_position - this->initial_integrated_forward_position) --->" << (ss.integrated_forward_position - this->initial_integrated_forward_position) << std::endl;  
        // std::cout << "this->linear_odom_scale -->" << this->linear_odom_scale << std::endl;
        
        float yaw_displacement = 
            (ss.integrated_turn_position - this->initial_integrated_turn_position) * 
            degrees_to_radians * this->angular_odom_scale;
        float yaw_rate = ss.yaw_rate * degrees_to_radians;
        // std::cout << "--> " << ss.yaw_rate << std::endl;
        // Test yaw angles values
        // std::cout << ss.yaw_rate << std::endl;
        
        float vel_x = 0.0;
        float vel_y = 0.0;
        float vel_w = 0.0;

        if (!this->first_odometry) {
            float delta_forward_displacement = forward_displacement - this->last_forward_displacement;
            float delta_yaw_displacement = yaw_displacement - this->last_yaw_displacement;
            double delta_time = (current_time - this->last_time).seconds();

            if (delta_time > 0.0) {
                // The robot's linear velocity (in its own frame) is simply the
                // rate of change of its forward displacement.
                vel_x = delta_forward_displacement / delta_time;
                vel_y = 0.0; // A differential drive robot has no sideways velocity.

                // Angular velocity is the rate of change of yaw displacement (encoder-based).
                vel_w = delta_yaw_displacement / delta_time;

                // The POSE calculation remains the same, as it IS in the odom frame.
                this->odometry_w = yaw_displacement;
                float delta_odometry_x = delta_forward_displacement * std::cos(this->odometry_w);
                this->odometry_x += delta_odometry_x;
                float delta_odometry_y = delta_forward_displacement * std::sin(this->odometry_w);
                this->odometry_y += delta_odometry_y;

                // Store the measured velocity for logging.
                this->measured_linear_vel = vel_x;
                this->measured_angular_vel = vel_w;

                // Print velocity information for debugging.
                double measured_vs_cmd_linear_diff = 0.0;
        if (std::abs(this->command_linear_vel) > 1e-6) { // Avoid division by zero
            measured_vs_cmd_linear_diff = ((this->measured_linear_vel - this->command_linear_vel) / this->command_linear_vel) * 100.0;
        }

        double sent_vs_cmd_linear_diff = 0.0;
        if (std::abs(this->command_linear_vel) > 1e-6) {
            sent_vs_cmd_linear_diff = ((this->sent_linear_vel - this->command_linear_vel) / this->command_linear_vel) * 100.0;
        }

        double measured_vs_cmd_angular_diff = 0.0;
        if (std::abs(this->command_angular_vel) > 1e-6) {
            measured_vs_cmd_angular_diff = ((this->measured_angular_vel - this->command_angular_vel) / this->command_angular_vel) * 100.0;
        }
        
        double sent_vs_cmd_angular_diff = 0.0;
        if (std::abs(this->command_angular_vel) > 1e-6) {
            sent_vs_cmd_angular_diff = ((this->sent_angular_vel - this->command_angular_vel) / this->command_angular_vel) * 100.0;
        }
        
        if (this->log_level == "debug")
                {
                    // Set formatting for floating point numbers
                    std::cout << std::fixed << std::setprecision(2);

                    // Print distance traveled.
                    std::cout << "Dist: " << forward_displacement << "m -- " << std::endl;

                    std::cout << "Velocities [Cmd | Sent | Measured] (% Diff from Cmd):" << std::endl;

                    // Print Linear Velocity info
                    std::cout << "-- Linear: ["
                            << this->command_linear_vel << "|"
                            << this->sent_linear_vel << "|"
                            << this->measured_linear_vel << "]"
                            // Switch to signed, 1-decimal-place formatting for percentages
                            << " (Sent: " << std::showpos << std::setprecision(1) 
                            << sent_vs_cmd_linear_diff << "%, Measured: "
                            << measured_vs_cmd_linear_diff << "%) m/s" << std::endl;

                    // Switch back to 2-decimal-place formatting for angular velocities
                    std::cout << std::noshowpos << std::setprecision(2)
                            << "-- Angular: ["
                            << this->command_angular_vel << "|"
                            << this->sent_angular_vel << "|"
                            << this->measured_angular_vel << "]"
                            // Switch to signed, 1-decimal-place formatting for percentages
                            << " (Sent: " << std::showpos << std::setprecision(1) 
                            << sent_vs_cmd_angular_diff << "%, Measured: "
                            << measured_vs_cmd_angular_diff << "%) rad/s"
                            // End the line and reset formatting
                            << std::endl;
                }
            }
        } else {
            this->first_odometry = false;
        }

        this->last_forward_displacement = forward_displacement;
        this->last_yaw_displacement = yaw_displacement;
        this->last_time = current_time;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, yaw_displacement); // Roll = 0, Pitch = 0, Yaw = yaw_displacement
        geometry_msgs::msg::Quaternion quat = tf2::toMsg(quat_tf);

        if (this->broadcast_tf) {
            this->odom_trans.header.stamp = current_time;
            this->odom_trans.transform.translation.x = this->odometry_x;
            this->odom_trans.transform.translation.y = this->odometry_y;
            this->odom_trans.transform.rotation = quat;
            
            this->odom_broadcaster->sendTransform(this->odom_trans);         
        }

        // Debug odometry --> start from here
        this->odom_msg.header.stamp = current_time;
        this->odom_msg.pose.pose.position.x = this->odometry_x;
        this->odom_msg.pose.pose.position.y = this->odometry_y;
        // this->odom_msg.pose.pose.orientation = quat;
        this->odom_msg.pose.pose.orientation.x = quat.x;
        this->odom_msg.pose.pose.orientation.y = quat.y;
        this->odom_msg.pose.pose.orientation.z = quat.z;
        this->odom_msg.pose.pose.orientation.w = quat.w;        
        
        this->odom_msg.twist.twist.linear.x = vel_x;
        this->odom_msg.twist.twist.linear.y = vel_y;
        this->odom_msg.twist.twist.angular.z = vel_w;
        // this->odom_msg.twist.twist.angular.w = this->odometry_w;

        this->odom_pub->publish(this->odom_msg);
    }

    /**
    * This method is called if a motor command is not received
    * within the segway_motor_timeout interval.  It halts the
    * robot for safety reasons.
    */
    void motor_timeoutCallback() {
        std::lock_guard<std::mutex> lock(this->m_mutex);
        //RCLCPP_INFO(this->get_logger(), "Motor command timeout! Setting target linear and angular velocities to be zero.");
        this->target_linear_vel = 0.0;
        this->target_angular_vel = 0.0;
        this->command_linear_vel = 0.0;
        this->command_angular_vel = 0.0;
        this->motor_timeout_timer->cancel();
    }

    /**
    * The handler for messages received on the 'cmd_vel' topic.
    */
    void cmd_velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        if (!this->connected)
            return;
    
        std::lock_guard<std::mutex> lock(this->m_mutex);

        // Access the velocity data through the 'twist' field.
        double x = msg->twist.linear.x;

        // Print the linear velocity to the terminal for debugging.
        RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel: linear.x=%f", x);

        double z = msg->twist.angular.z;

        if (this->invert_x) {
            x *= -1;
        }
        if (this->invert_z) {
            z *= -1;
        }

        // Check for maximum linear velocity.
        if (this->max_linear_vel != 0.0) {
            if (std::abs(x) > this->max_linear_vel) {
                x = (x > 0) ? this->max_linear_vel : -this->max_linear_vel;
            }
        }

        // Check for maximum angular velocity.
        if (this->max_angular_vel != 0.0) {
            if (std::abs(z) > this->max_angular_vel) {
                z = (z > 0) ? this->max_angular_vel : -this->max_angular_vel;
            }
        }

        this->target_linear_vel = x;
        this->target_angular_vel = z * radians_to_degrees;  // Convert to degrees.

        // Store the commanded velocities for logging.
        this->command_linear_vel = x;
        this->command_angular_vel = z;

        // Create or reset the motor timeout timer.
        if (this->motor_timeout_timer != nullptr) {
            this->motor_timeout_timer->reset();
        } else {
            this->motor_timeout_timer = this->create_wall_timer(
                std::chrono::duration<double>(this->segway_motor_timeout),
                std::bind(&SegwayRMPNode::motor_timeoutCallback, this)
            );
        }
    }

private:
    // Functions
    void setupROSComms() {
        // Subscribe to command velocities
        this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel",
            10,
            std::bind(&SegwayRMPNode::cmd_velCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "cmd_vel subscriber initialized.");
        // Advertise the SegwayStatusStamped
        this->segway_status_pub = this->create_publisher<segway_rmp_ros2::msg::SegwayStatusStamped>("segway_status", 10);

        // Advertise the Odometry Msg
        this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    }

    void setupSegwayRMP() {
        std::stringstream ss;
        ss << "Connecting to Segway RMP via ";
        this->segway_rmp = new segwayrmp::SegwayRMP(this->interface_type, this->segway_rmp_type);
        if (this->interface_type_str == "serial") {
            ss << "serial on serial port: " << this->serial_port;
            this->segway_rmp->configureSerial(this->serial_port);
        } else if (this->interface_type_str == "usb") {
            ss << "usb ";
            if (this->usb_selector == "serial_number") {
                ss << "identified by the device serial number: " << this->serial_number;
                this->segway_rmp->configureUSBBySerial(this->serial_number);
            }
            if (this->usb_selector == "description") {
                ss << "identified by the device description: " << this->usb_description;
                this->segway_rmp->configureUSBByDescription(this->usb_description);
            }
            if (this->usb_selector == "index") {
                ss << "identified by the device index: " << this->usb_index;
                this->segway_rmp->configureUSBByIndex(this->usb_index);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        
        // Set the instance variable (if needed)
        segwayrmp_node_instance = this;

        // Set callbacks for segway data and messages
        this->segway_rmp->setStatusCallback(handleStatusWrapper);
        this->segway_rmp->setLogMsgCallback("debug", handleDebugMessages);
        this->segway_rmp->setLogMsgCallback("info", handleInfoMessages);
        this->segway_rmp->setLogMsgCallback("error", handleErrorMessages);
    }

    int getParameters() {
        // Get Interface Type
        this->declare_parameter<std::string>("interface_type", "serial");
        this->get_parameter("interface_type", this->interface_type_str);

        // Get Configurations based on Interface Type
        if (this->interface_type_str == "serial") {
            this->interface_type = segwayrmp::serial;
            this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
            this->get_parameter("serial_port", this->serial_port);
        } else if (this->interface_type_str == "usb") {
            this->interface_type = segwayrmp::usb;
            this->declare_parameter<std::string>("usb_selector", "index");
            this->get_parameter("usb_selector", this->usb_selector);

            if (this->usb_selector == "index") {
                this->declare_parameter<int>("usb_index", 0);
                this->get_parameter("usb_index", this->usb_index);
            } else if (this->usb_selector == "serial_number") {
                this->declare_parameter<std::string>("usb_serial_number", "00000000");
                this->get_parameter("usb_serial_number", this->serial_number);
                if (this->serial_number == "00000000") {
                    RCLCPP_WARN(this->get_logger(), "The serial_number parameter is set to the default 00000000, which shouldn't work.");
                }
            } else if (this->usb_selector == "description") {
                this->declare_parameter<std::string>("usb_description", "Robotic Mobile Platform");
                this->get_parameter("usb_description", this->usb_description);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "Invalid USB selector: %s, valid types are 'index', 'serial_number', and 'description'.", 
                    this->usb_selector.c_str());
                return 1;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid interface type: %s, valid interface types are 'serial' and 'usb'.",
                this->interface_type_str.c_str());
            return 1;
        }

        // Get Setup Motor Timeout
        this->declare_parameter<double>("motor_timeout", 0.5);
        this->get_parameter("motor_timeout", this->segway_motor_timeout);

        // Get frame id parameter
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->get_parameter("frame_id", this->frame_id);
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->get_parameter("odom_frame_id", this->odom_frame_id);
        
        this->sss_msg.header.frame_id = this->frame_id;
        this->odom_trans.header.frame_id = this->odom_frame_id;
        this->odom_trans.child_frame_id = this->frame_id;
        this->odom_msg.header.frame_id = this->odom_frame_id;
        this->odom_msg.child_frame_id = this->frame_id;

        // Get cmd_vel inversion parameters
        this->declare_parameter<bool>("invert_linear_vel_cmds", false);
        this->get_parameter("invert_linear_vel_cmds", this->invert_x);
        this->declare_parameter<bool>("invert_angular_vel_cmds", false);
        this->get_parameter("invert_angular_vel_cmds", this->invert_z);

        // Get option for enable/disable tf broadcasting
        this->declare_parameter<bool>("broadcast_tf", true);
        this->get_parameter("broadcast_tf", this->broadcast_tf);

        // Get the segway rmp type
        std::string segway_rmp_type_str;
        this->declare_parameter<std::string>("rmp_type", "200/400");
        this->get_parameter("rmp_type", segway_rmp_type_str);
        
        if (segway_rmp_type_str == "200/400") {
            this->segway_rmp_type = segwayrmp::rmp200;
        } else if (segway_rmp_type_str == "50/100") {
            this->segway_rmp_type = segwayrmp::rmp100;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid rmp type: %s, valid rmp types are '200/400' and '50/100'.",
                segway_rmp_type_str.c_str());
            return 1;
        }

        RCLCPP_INFO(this->get_logger(), "Interface: %s, RMP Type: %s", 
            this->interface_type_str.c_str(), segway_rmp_type_str.c_str());

        // Get the linear acceleration limits in m/s^2. Zero means infinite.
        this->declare_parameter<double>("linear_pos_accel_limit", 0.0);
        this->get_parameter("linear_pos_accel_limit", this->linear_pos_accel_limit);
        this->declare_parameter<double>("linear_neg_accel_limit", 0.0);
        this->get_parameter("linear_neg_accel_limit", this->linear_neg_accel_limit);

        // Get the angular acceleration limits in deg/s^2. Zero means infinite.
        this->declare_parameter<double>("angular_pos_accel_limit", 0.0);
        this->get_parameter("angular_pos_accel_limit", this->angular_pos_accel_limit);
        this->declare_parameter<double>("angular_neg_accel_limit", 0.0);
        this->get_parameter("angular_neg_accel_limit", this->angular_neg_accel_limit);
        
        // Check for valid acceleration limits
        if (this->linear_pos_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid linear positive acceleration limit of %f (must be non-negative).",
                this->linear_pos_accel_limit);
            return 1;
        }
        if (this->linear_neg_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid linear negative acceleration limit of %f (must be non-negative).",
                this->linear_neg_accel_limit);
            return 1;
        }
        if (this->angular_pos_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid angular positive acceleration limit of %f (must be non-negative).",
                this->angular_pos_accel_limit);
            return 1;
        }
        if (this->angular_neg_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid angular negative acceleration limit of %f (must be non-negative).",
                this->angular_neg_accel_limit);
            return 1;
        }

        RCLCPP_INFO(this->get_logger(), "Accel limits: linear: pos = %f, neg = %f, angular: pos = %f, neg = %f.",
            this->linear_pos_accel_limit, this->linear_neg_accel_limit, 
            this->angular_pos_accel_limit, this->angular_neg_accel_limit);

        // Get velocity limits. Zero means no limit
        this->declare_parameter<double>("max_linear_vel", 0.0);
        this->get_parameter("max_linear_vel", this->max_linear_vel);
        this->declare_parameter<double>("max_angular_vel", 0.0);
        this->get_parameter("max_angular_vel", this->max_angular_vel);
        
        if (this->max_linear_vel < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid max linear velocity limit of %f (must be non-negative).",
                this->max_linear_vel);
            return 1;
        }

        if (this->max_angular_vel < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid max angular velocity limit of %f (must be non-negative).",
                this->max_angular_vel);
            return 1;
        }

        RCLCPP_INFO(this->get_logger(), "Velocity limits: linear: %f, angular: %f.",
            this->max_linear_vel, this->max_angular_vel); 

        // Convert the linear acceleration limits to have units of (m/s^2)/20
        this->linear_pos_accel_limit /= 20;
        this->linear_neg_accel_limit /= 20;

        // Convert the angular acceleration limits to have units of (deg/s^2)/20
        this->angular_pos_accel_limit /= 20;
        this->angular_neg_accel_limit /= 20;

        // Get the scale correction parameters for odometry
        this->declare_parameter<double>("linear_odom_scale", 1.0);
        this->get_parameter("linear_odom_scale", this->linear_odom_scale);
        this->declare_parameter<double>("angular_odom_scale", 1.0);
        this->get_parameter("angular_odom_scale", this->angular_odom_scale);

        // Check if a software odometry reset is required
        this->declare_parameter<bool>("reset_odometry", false);
        this->get_parameter("reset_odometry", this->reset_odometry);
        this->declare_parameter<double>("odometry_reset_duration", 1.0);
        this->get_parameter("odometry_reset_duration", this->odometry_reset_duration);

        // Get the scale correction parameters for velocities.
        this->declare_parameter<bool>("linear_vel_scale", true);
        this->get_parameter("linear_vel_scale", this->linear_vel_scale);
        this->declare_parameter<bool>("angular_vel_scale", true);
        this->get_parameter("angular_vel_scale", this->angular_vel_scale);

        // Initialize velocity trackers.
        this->command_linear_vel = 0.0;
        this->command_angular_vel = 0.0;
        this->sent_linear_vel = 0.0;
        this->sent_angular_vel = 0.0;
        this->measured_linear_vel = 0.0;
        this->measured_angular_vel = 0.0;

        // Get the log type parameter.
        this->declare_parameter<std::string>("log_level", "info");
        this->get_parameter("log_level", this->log_level);
        if (this->log_level != "info" && this->log_level != "debug" && this->log_level != "error") {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid log type: %s, valid log types are 'info', 'debug', and 'error'.",
                this->log_level.c_str());
            return 1;
        }

        RCLCPP_INFO(this->get_logger(), "Odometry scales: linear = %f, angular = %f.",
            this->linear_odom_scale, this->angular_odom_scale);

        RCLCPP_INFO(this->get_logger(), "Apply velocity scales? linear = %s, angular = %s.",
            this->linear_vel_scale ? "true" : "false", this->angular_vel_scale ? "true" : "false");

        return 0;
    }

    // Variables
    rclcpp::Node::SharedPtr node;  // Shared pointer for the ROS2 node

    rclcpp::TimerBase::SharedPtr keep_alive_timer;  // Timer for keep-alive functionality

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber;  // Subscriber for cmd_vel
    rclcpp::Publisher<segway_rmp_ros2::msg::SegwayStatusStamped>::SharedPtr segway_status_pub;  // Publisher for Segway status
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;  // Publisher for odometry
    // Nikunj
    // tf2_ros::TransformBroadcaster odom_broadcaster;  // Transform broadcaster for odometry
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    segwayrmp::SegwayRMP *segway_rmp;  // Pointer to the Segway RMP object

    // Parameter variables
    std::string interface_type_str;  // Interface type as a string
    segwayrmp::InterfaceType interface_type;  // Enum for interface type
    segwayrmp::SegwayRMPType segway_rmp_type;  // Enum for Segway RMP type
    std::string serial_port;  // Serial port string
    std::string usb_selector;  // USB selector string
    std::string serial_number;  // Serial number string
    std::string usb_description;  // USB description string
    int usb_index;  // USB index

    double segway_motor_timeout;  // Motor timeout duration
    rclcpp::TimerBase::SharedPtr motor_timeout_timer;  // Timer for motor timeout functionality

    std::string frame_id;  // Frame ID for TF
    std::string odom_frame_id;  // Odometry frame ID
    bool invert_x;  // Inversion for linear velocity commands
    bool invert_z;  // Inversion for angular velocity commands
    bool broadcast_tf;  // Flag for broadcasting TF

    double linear_vel;  // Current linear velocity
    double angular_vel;  // Current angular velocity in deg/s

    double target_linear_vel;  // Target linear velocity in m/s
    double target_angular_vel;  // Target angular velocity in deg/s

    double linear_pos_accel_limit;  // Max linear acceleration in (m/s^2)/20
    double linear_neg_accel_limit;  // Max linear deceleration in (m/s^2)/20
    double angular_pos_accel_limit;  // Max angular acceleration in (deg/s^2)/20
    double angular_neg_accel_limit;  // Max angular deceleration in (deg/s^2)/20

    double linear_odom_scale;  // Linear odometry scale correction
    double angular_odom_scale;  // Angular odometry scale correction

    bool linear_vel_scale;  // Linear velocity scale correction
    bool angular_vel_scale;  // Angular velocity scale correction

    double max_linear_vel;  // Maximum allowed linear velocity
    double max_angular_vel;  // Maximum allowed angular velocity

    bool connected;  // Connection status

    segway_rmp_ros2::msg::SegwayStatusStamped sss_msg;  // Segway status message
    geometry_msgs::msg::TransformStamped odom_trans;  // Transform message for odometry
    nav_msgs::msg::Odometry odom_msg;  // Odometry message

    int count;  // Counter variable

    bool first_odometry;  // Flag for the first odometry update
    float last_forward_displacement;  // Last forward displacement
    float last_yaw_displacement;  // Last yaw displacement
    float odometry_x;  // Odometry x position
    float odometry_y;  // Odometry y position
    float odometry_w;  // Odometry yaw orientation
    rclcpp::Time last_time;  // Last time for odometry calculations

    std::mutex m_mutex;  // Mutex for thread safety

    // Hardware reset of integrators can sometimes fail.
    // These help in performing a software reset.
    bool reset_odometry;  // Flag for resetting odometry
    double odometry_reset_duration;  // Duration for odometry reset
    rclcpp::Time odometry_reset_start_time;  // Start time for odometry reset
    double initial_integrated_forward_position;  // Initial integrated forward position
    double initial_integrated_left_wheel_position;  // Initial integrated left wheel position
    double initial_integrated_right_wheel_position;  // Initial integrated right wheel position
    double initial_integrated_turn_position;  // Initial integrated turn position

    std::string log_level;  // Log message type

    // Variables to track the different velocities.
    double command_linear_vel;
    double command_angular_vel;
    double sent_linear_vel;
    double sent_angular_vel;
    double measured_linear_vel;
    double measured_angular_vel;
};

// Callback wrapper
void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss) {
    segwayrmp_node_instance->handleStatus(ss);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS2 system

    // Create a shared pointer for the node
    auto segway_node = std::make_shared<SegwayRMPNode>();

    // Call the run() method (sping is within run)
    segway_node->run();  // Assuming run() is a public method of SegwayRMPNode

    rclcpp::shutdown();  // Shutdown the ROS2 system
    return 0; 
}
