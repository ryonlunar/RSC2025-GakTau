#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uav_interfaces/action/rescue_mission.hpp"
#include "uav_interfaces/msg/drone_status.hpp"

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node {
public:
    using RescueMission = uav_interfaces::action::RescueMission;
    using GoalHandleRescueMission = rclcpp_action::ServerGoalHandle<RescueMission>;

    DroneController() : Node("drone_controller") {

        //ACTIONS
        this->action_server_ = rclcpp_action::create_server<RescueMission>(
            this, "/rescue_mission",
            std::bind(&DroneController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DroneController::handle_cancel, this, std::placeholders::_1),
            std::bind(&DroneController::handle_accepted, this, std::placeholders::_1)
        );

        //TOPICS
        drone_position_publisher_ = this->create_publisher<uav_interfaces::msg::DroneStatus>("drone_position", 10);
        drone_status_publisher_ = this->create_publisher<uav_interfaces::msg::DroneStatus>("drone_status", 10);

        //PARAMS
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

    //ACTIONS
    rclcpp_action::Server<RescueMission>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const RescueMission::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Menerima permintaan rescue ke (%d, %d)", goal->target_x, goal->target_y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRescueMission> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Misi rescue dibatalkan");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleRescueMission> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Menjalankan rescue mission...");
        auto result = std::make_shared<RescueMission::Result>();
        result->success = true;
        result->message = "Misi selesai!";
        goal_handle->succeed(result);
    }

    //TOPICS
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<uav_interfaces::msg::DroneStatus>::SharedPtr drone_position_publisher_;
    rclcpp::Publisher<uav_interfaces::msg::DroneStatus>::SharedPtr drone_status_publisher_;
    uav_interfaces::msg::DroneStatus drone_message_;

    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Publishing: drone position and drone status");
        drone_position_publisher_->publish(drone_message_);
        drone_status_publisher_->publish(drone_message_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}