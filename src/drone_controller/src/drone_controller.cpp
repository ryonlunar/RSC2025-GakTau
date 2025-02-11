#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uav_interfaces/action/rescue_mission.hpp"

class DroneController : public rclcpp::Node {
public:
    using RescueMission = uav_interfaces::action::RescueMission;
    using GoalHandleRescueMission = rclcpp_action::ServerGoalHandle<RescueMission>;

    DroneController() : Node("drone_controller") {
        this->action_server_ = rclcpp_action::create_server<RescueMission>(
            this, "/rescue_mission",
            std::bind(&DroneController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DroneController::handle_cancel, this, std::placeholders::_1),
            std::bind(&DroneController::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
