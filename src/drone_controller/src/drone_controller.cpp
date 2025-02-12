#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uav_interfaces/action/rescue_mission.hpp"
#include "uav_interfaces/msg/drone_status.hpp"
#include "uav_interfaces/msg/flag.hpp"
#include "uav_interfaces/srv/start_simulation.hpp"
#include "uav_interfaces/srv/stop_simulation.hpp"

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node {
public:

    using RescueMission = uav_interfaces::action::RescueMission;
    using GoalHandleRescueMission = rclcpp_action::ServerGoalHandle<RescueMission>;

    DroneController() : Node("drone_controller") {

        RCLCPP_INFO(this->get_logger(), "Silahkan jalankan simulasi atau aksi");

        //TOPICS
        drone_position_publisher_ = this->create_publisher<uav_interfaces::msg::DroneStatus>("drone_position", 10);
        drone_status_publisher_ = this->create_publisher<uav_interfaces::msg::DroneStatus>("drone_status", 10);
        flag_pub_ = this->create_publisher<uav_interfaces::msg::Flag>("flag", 10);
        flag_sub_ = this->create_subscription<uav_interfaces::msg::Flag>(
            "flag", 10, std::bind(&DroneController::found_path_callback, this, std::placeholders::_1)
        );

        //PARAMS
        float initial_energy = this->declare_parameter<float>("initial_energy", 200.0);
        int initial_passengers = this->declare_parameter<int>("initial_passengers", 5);
        float initial_x = this->declare_parameter<float>("initial_x", 1.0);
        float initial_y = this->declare_parameter<float>("initial_y", 5.0);
        bool initial_sim_flag = this->declare_parameter<bool>("initial_sim_flag", false);
        bool initial_found_path_flag = this->declare_parameter<bool>("initial_found_path_flag", false);
        
        //INITIALIZATION
        drone_message_.position.x = initial_x;
        drone_message_.position.y = initial_y;
        drone_message_.energy = initial_energy;
        drone_message_.passengers = initial_passengers;
        flag_message_.simulation_flag = initial_sim_flag;
        flag_message_.found_path_flag = initial_found_path_flag;

        //SERVICE
        start_simulation_service_ = this->create_service<uav_interfaces::srv::StartSimulation>(
            "/start_simulation",
            std::bind(&DroneController::handle_start_simulation, this, std::placeholders::_1, std::placeholders::_2)
        );
        stop_simulation_service_ = this->create_service<uav_interfaces::srv::StopSimulation>(
            "/stop_simulation",
            std::bind(&DroneController::handle_stop_simulation, this, std::placeholders::_1, std::placeholders::_2)
        );

        //ACTIONS
        this->action_server_ = rclcpp_action::create_server<RescueMission>(
            this, "/rescue_mission",
            std::bind(&DroneController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DroneController::handle_cancel, this, std::placeholders::_1),
            std::bind(&DroneController::handle_accepted, this, std::placeholders::_1)
        );
    }

private:

    //TOPIC drone_position dan drone_status + TIMER
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<uav_interfaces::msg::DroneStatus>::SharedPtr drone_position_publisher_;
    rclcpp::Publisher<uav_interfaces::msg::DroneStatus>::SharedPtr drone_status_publisher_;
    uav_interfaces::msg::DroneStatus drone_message_;
    uav_interfaces::msg::DroneStatus drone_message_sim_;

    void timer_callback() {

        RCLCPP_INFO(this->get_logger(), "Publishing: drone position: (%f %f) | drone status: energi: %f penummpang: %d",
            drone_message_.position.x, drone_message_.position.y, drone_message_.energy, drone_message_.passengers);
        drone_position_publisher_->publish(drone_message_);
        drone_status_publisher_->publish(drone_message_);
        flag_pub_->publish(flag_message_);
    }

    //TOPIC flag
    rclcpp::Publisher<uav_interfaces::msg::Flag>::SharedPtr flag_pub_;
    rclcpp::Subscription<uav_interfaces::msg::Flag>::SharedPtr flag_sub_;
    uav_interfaces::msg::Flag flag_message_;

    void found_path_callback(const uav_interfaces::msg::Flag::SharedPtr msg) {
        flag_message_.simulation_flag = msg->simulation_flag;
        flag_message_.found_path_flag = msg->found_path_flag;
        //gerakan drone
    }

    //SERVICE /start_simulation dan /stop_simulation
    rclcpp::Service<uav_interfaces::srv::StartSimulation>::SharedPtr start_simulation_service_;
    rclcpp::Service<uav_interfaces::srv::StopSimulation>::SharedPtr stop_simulation_service_;

    void handle_start_simulation(
        const std::shared_ptr<uav_interfaces::srv::StartSimulation::Request> request,
        std::shared_ptr<uav_interfaces::srv::StartSimulation::Response> response
    ) {
        (void) request; // tidak digunakan
        flag_message_.simulation_flag = true;
        drone_message_sim_ = drone_message_; //menyimpan nilai initial 
        RCLCPP_INFO(this->get_logger(), "Menginisiasi nilai awal:\n" 
            "energi awal : %f\npenumpang awal : %d\nKoordinat awal : (%f, %f)", drone_message_.energy, drone_message_.passengers,
            drone_message_.position.x, drone_message_.position.y);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Memulai simulasi");

        // mekanisme untuk gagal simulasi apa ya? atau gausah?

        response->success = true;
        response->message = "Melakukan simulasi";

        timer_ = this->create_wall_timer(
            500ms, std::bind(&DroneController::timer_callback, this)
        );
    }

    void handle_stop_simulation(
        const std::shared_ptr<uav_interfaces::srv::StopSimulation::Request> request,
        std::shared_ptr<uav_interfaces::srv::StopSimulation::Response> response
    ) {
        (void) request; // tidak digunakan
        flag_message_.simulation_flag = false;
        flag_message_.found_path_flag = false;
        flag_pub_->publish(flag_message_);
        response->success = true;
        response->message = "Simulasi dihentikan";
        RCLCPP_INFO(this->get_logger(), response->message.c_str());
        timer_->cancel();
        drone_message_ = drone_message_sim_; // ambil ulang nilai initial
    }

    //ACTIONS
    rclcpp_action::Server<RescueMission>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const RescueMission::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Menerima permintaan rescue ke (%d, %d)", goal->target_x, goal->target_y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleRescueMission> goal_handle) {
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
