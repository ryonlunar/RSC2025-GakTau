#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/drone_status.hpp"

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node
{
    public:
        DroneController()
        : Node("drone_controller")
        {   
            drone_position_publisher_ = this->create_publisher<uav_interfaces::msg::DroneStatus>("drone_position", 10);
            drone_status_publisher_ = this->create_publisher<uav_interfaces::msg::DroneStatus>("drone_status", 10);

            float initial_energy = this->declare_parameter<float>("initial_energy", 200.0);
            int initial_passengers = this->declare_parameter<int>("initial_passengers", 5);
            float initial_x = this->declare_parameter<float>("initial_x", 1.0);
            float initial_y = this->declare_parameter<float>("initial_y", 5.0);
            
            drone_message_.position.x = initial_x;
            drone_message_.position.y = initial_y;
            drone_message_.energy = initial_energy;
            drone_message_.passengers = initial_passengers;
            
            timer_ = this->create_wall_timer(
                500ms, std::bind(&DroneController::timer_callback, this)
            );
        }
    private:
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Publishing: drone position and drone status");
            drone_position_publisher_->publish(drone_message_);
            drone_status_publisher_->publish(drone_message_);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<uav_interfaces::msg::DroneStatus>::SharedPtr drone_position_publisher_;
        rclcpp::Publisher<uav_interfaces::msg::DroneStatus>::SharedPtr drone_status_publisher_;
        uav_interfaces::msg::DroneStatus drone_message_;
};      

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
