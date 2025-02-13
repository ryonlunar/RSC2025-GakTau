#include <memory>
#include <string>
#include <iostream>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/map_state.hpp"
#include "uav_interfaces/msg/drone_status.hpp"
#include "uav_interfaces/msg/flag.hpp"
#include "uav_interfaces/srv/add_victim.hpp"
#include "uav_interfaces/srv/add_obstacle.hpp"
#include "uav_interfaces/srv/update_drone_position.hpp"  // Include service to update drone position
#include "queue_obs.hpp"
#include "queue_vic.hpp"

using namespace std::chrono_literals;

class MapManager : public rclcpp::Node
{
public:
    MapManager()
        : Node("map_manager")
    {
        //TOPIC drone_status, drone_position
        drone_status_sub_ = this->create_subscription<uav_interfaces::msg::DroneStatus>(
            "drone_status", 10, std::bind(&MapManager::drone_status_callback, this, std::placeholders::_1));

        drone_position_sub_ = this->create_subscription<uav_interfaces::msg::DroneStatus>(
            "drone_position", 10, std::bind(&MapManager::drone_position_callback, this, std::placeholders::_1));

        //TOPIC map_state
        map_state_pub_ = this->create_publisher<uav_interfaces::msg::MapState>("map_state", 10);

        //SERVICE /add_victim, /add_obstacle
        add_victim_service_ = this->create_service<uav_interfaces::srv::AddVictim>(
            "/add_victim", std::bind(&MapManager::handle_add_victim, this, std::placeholders::_1, std::placeholders::_2));

        add_obstacle_service_ = this->create_service<uav_interfaces::srv::AddObstacle>(
            "/add_obstacle", std::bind(&MapManager::handle_add_obstacle, this, std::placeholders::_1, std::placeholders::_2));

        // Create client to call update_drone_position service
        update_drone_position_client_ = this->create_client<uav_interfaces::srv::UpdateDronePosition>(
            "/update_drone_position");

        //TOPIC FLAG
        flag_sub_ = this->create_subscription<uav_interfaces::msg::Flag>(
            "flag", 10, std::bind(&MapManager::flag_callback, this, std::placeholders::_1));

        flag_message_.simulation_flag = false;
        flag_message_.found_path_flag = false;

        // Initialize map state
        int initial_width = this->declare_parameter<int>("initial_width", 7);
        int initial_height = this->declare_parameter<int>("initial_height", 7);
        map_state_message_.width = initial_width;
        map_state_message_.height = initial_height;

        // Initialize grid data and path numbers
        map_state_message_.grid_data.resize(initial_width * initial_height, 0); // Empty grid
        map_state_message_.path_numbers.resize(initial_width * initial_height, 0);

        // Initialize sample grid values
        map_state_message_.grid_data[0] = 4;  // victim
        map_state_message_.grid_data[8] = 4;  // victim
        map_state_message_.grid_data[15] = 3; // obstacle
        map_state_message_.grid_data[16] = 3; // obstacle
        map_state_message_.grid_data[17] = 3; // obstacle

        // Publish initial map state
        map_state_pub_->publish(map_state_message_);

        // Initialize a timer to publish updated map state
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MapManager::publish_map_state, this));
    }

private:
    // Topic and services
    rclcpp::Publisher<uav_interfaces::msg::MapState>::SharedPtr map_state_pub_;
    rclcpp::Subscription<uav_interfaces::msg::DroneStatus>::SharedPtr drone_status_sub_;
    rclcpp::Subscription<uav_interfaces::msg::DroneStatus>::SharedPtr drone_position_sub_;
    rclcpp::Subscription<uav_interfaces::msg::Flag>::SharedPtr flag_sub_;

    rclcpp::Service<uav_interfaces::srv::AddVictim>::SharedPtr add_victim_service_;
    rclcpp::Service<uav_interfaces::srv::AddObstacle>::SharedPtr add_obstacle_service_;
    rclcpp::Client<uav_interfaces::srv::UpdateDronePosition>::SharedPtr update_drone_position_client_;  // Client to call the service

    rclcpp::TimerBase::SharedPtr timer_;  // Timer to periodically publish the map state

    uav_interfaces::msg::MapState map_state_message_;
    uav_interfaces::msg::Flag flag_message_;

    // Handle updates to drone position
    void handle_update_drone_position(
        const std::shared_ptr<uav_interfaces::srv::UpdateDronePosition::Request> request,
        std::shared_ptr<uav_interfaces::srv::UpdateDronePosition::Response> response)
    {
        int x = request->x;
        int y = request->y;

        // Validate the coordinates
        size_t idx = static_cast<size_t>(y) * map_state_message_.width + static_cast<size_t>(x);
        if (idx >= map_state_message_.grid_data.size()) {
            response->success = false;
            response->message = "Koordinat drone di luar batas peta!";
            return;
        }

        // Update the grid data with drone position (e.g., use 1 for drone)
        map_state_message_.grid_data[idx] = 1;  // Mark the drone position in the map

        // Publish updated map
        map_state_pub_->publish(map_state_message_);

        response->success = true;
        response->message = "Posisi drone berhasil diperbarui!";
    }

    void drone_status_callback(const uav_interfaces::msg::DroneStatus::SharedPtr drone_message) const
    {
        // Print drone status
        std::cout << "Energi: " << drone_message->energy << std::endl;
        std::cout << "Penumpang: " << drone_message->passengers << std::endl;
    }

    void drone_position_callback(const uav_interfaces::msg::DroneStatus::SharedPtr drone_message)
    {
        // Call the service to update drone position on the map
        if (update_drone_position_client_->wait_for_service(1s)) {
            auto request = std::make_shared<uav_interfaces::srv::UpdateDronePosition::Request>();
            request->x = drone_message->position.x;
            request->y = drone_message->position.y;
            auto future = update_drone_position_client_->async_send_request(request);
            
            // Handle response asynchronously if needed
            future.wait();
        }
    }

    void flag_callback(const uav_interfaces::msg::Flag::SharedPtr msg)
    {
        flag_message_.simulation_flag = msg->simulation_flag;
        flag_message_.found_path_flag = msg->found_path_flag;
    }

    // Handle adding a victim to the map
    void handle_add_victim(
        const std::shared_ptr<uav_interfaces::srv::AddVictim::Request> request,
        std::shared_ptr<uav_interfaces::srv::AddVictim::Response> response)
    {
        size_t idx = static_cast<size_t>(request->y) * map_state_message_.width + static_cast<size_t>(request->x);

        if (idx >= map_state_message_.grid_data.size()) {
            response->success = false;
            response->message = "Koordinat korban di luar batas peta!";
            return;
        }

        map_state_message_.grid_data[idx] = 4;  // Mark victim on the map (4 is the victim marker)
        map_state_pub_->publish(map_state_message_);
        response->success = true;
        response->message = "Korban berhasil ditambahkan!";
    }

    // Handle adding an obstacle to the map
    void handle_add_obstacle(
        const std::shared_ptr<uav_interfaces::srv::AddObstacle::Request> request,
        std::shared_ptr<uav_interfaces::srv::AddObstacle::Response> response)
    {
        size_t idx = static_cast<size_t>(request->y) * map_state_message_.width + static_cast<size_t>(request->x);

        if (idx >= map_state_message_.grid_data.size()) {
            response->success = false;
            response->message = "Koordinat obstacle di luar batas peta!";
            return;
        }

        map_state_message_.grid_data[idx] = 3;  // Mark obstacle on the map (3 is the obstacle marker)
        map_state_pub_->publish(map_state_message_);
        response->success = true;
        response->message = "Obstacle berhasil ditambahkan!";
    }

    // Publish the map state periodically
    void publish_map_state()
    {
        // Print the map state to the terminal
        std::cout << "=== Grid Map ===" << std::endl;
        int width = map_state_message_.width;
        int height = map_state_message_.height;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;

                if (map_state_message_.grid_data[idx] == 2) std::cout << "s ";  // Start
                else if (map_state_message_.grid_data[idx] == 4) std::cout << "k "; // Korban
                else if (map_state_message_.grid_data[idx] == 3) std::cout << "o "; // Obstacle
                else if (map_state_message_.grid_data[idx] == 1) std::cout << "d "; // drone
                else std::cout << "- "; // Empty space
            }
            std::cout << std::endl;
        }
        std::cout << "================" << std::endl;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManager>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // Run callbacks in multiple threads

    rclcpp::shutdown();
    return 0;
}