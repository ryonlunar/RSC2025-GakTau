#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/map_state.hpp"
#include "uav_interfaces/msg/drone_status.hpp"


class MapManager : public rclcpp::Node
{
    public:
        MapManager()
        : Node("map_manager")
        {
           map_state_pub_ = this->create_publisher<uav_interfaces::msg::MapState>("map_state", 10);
        
            int initial_width = this->declare_parameter<int>("initial_width", 7);
            int initial_height = this->declare_parameter<int>("initial_height", 7);
            map_state_message_.width = initial_width;
            map_state_message_.height = initial_height;
            
            std::vector<int64_t> initial_grid_data_64 = this->declare_parameter<std::vector<int64_t>>(
                "initial_grid_data", std::vector<int64_t>(initial_width*initial_height, 0) // Grid 8x8
            );
        
            std::vector<int64_t> initial_path_numbers_64 = this->declare_parameter<std::vector<int64_t>>(
                "initial_path_numbers", std::vector<int64_t>(initial_width*initial_height, 0) // Path numbers 8x8
            );

            // Konversi ke std::vector<int>
            std::vector<int> initial_grid_data(initial_grid_data_64.begin(), initial_grid_data_64.end());
            std::vector<int> initial_path_numbers(initial_path_numbers_64.begin(), initial_path_numbers_64.end());

            // simpan dalam member variable
            map_state_message_.grid_data = initial_grid_data;
            map_state_message_.path_numbers = initial_path_numbers;

            //contoh
            map_state_message_.grid_data[0] = 4;
            map_state_message_.grid_data[8] = 4;
            map_state_message_.grid_data[15] = 3;
            map_state_message_.grid_data[16] = 3;
            map_state_message_.grid_data[17] = 3;
            
           drone_status_sub_= this->create_subscription<uav_interfaces::msg::DroneStatus>(
                "drone_status", 10, std::bind(&MapManager::drone_status_callback, this, std::placeholders::_1)
            );

            drone_position_sub_= this->create_subscription<uav_interfaces::msg::DroneStatus>(
                "drone_position", 10, std::bind(&MapManager::drone_position_callback, this, std::placeholders::_1)
            );

            path_visualization_sub_ = this->create_subscription<uav_interfaces::msg::MapState>(
                "path_visualization", 10, std::bind(&MapManager::path_visualization_callback, this, std::placeholders::_1)
            );
        }

    private:
        void drone_status_callback(const uav_interfaces::msg::DroneStatus::SharedPtr drone_message) const
        {   
            std::cout << "energi = " << drone_message->energy << std::endl;
            std::cout << "penumpang = " << drone_message->passengers << std::endl << std::endl;
           map_state_pub_->publish(map_state_message_);
        }

        void drone_position_callback(const uav_interfaces::msg::DroneStatus::SharedPtr drone_message)
        {   
            // untuk ubah posisi drone lama jadi angka dihandle drone_controller 
            // if (previous_x != drone_message->position.x && previous_y != drone_message->position.y){
            //     map_state_message_.grid_data[previous_x * previous_y] = 9999; //posisi drone lama ditinggalkan menjadi path numbers
            //     map_state_message_.grid_data[drone_message->position.x * drone_message->position.y] = 2; //posisi drone baru
            //     previous_x = drone_message->position.x;
            //     previous_y = drone_message->position.y;
            // }
            map_state_message_.grid_data[drone_message->position.x * drone_message->position.y] = 2; //posisi drone baru

        }

        void path_visualization_callback(const uav_interfaces::msg::MapState::SharedPtr msg) {
            map_state_message_.width = msg->width;
            map_state_message_.height = msg->height;
            map_state_message_.grid_data = msg->grid_data;
            map_state_message_.path_numbers = msg->path_numbers;
    
            std::cout << "=== Grid Map ===" << std::endl;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int idx = y * width + x;
                    
                    if (msg->grid_data[idx] == 2) std::cout << "s ";  // Start
                    else if (msg->grid_data[idx] == 4) std::cout << "k "; // Korban
                    else if (msg->grid_data[idx] == 3) std::cout << "o "; // Obstacle
                    else if (msg->grid_data[idx] == 1) std::cout << "d "; // drone
                    else if (msg->grid_data[idx] > 0) std::cout << msg->path_numbers[idx] << " "; // Jalur drone
                    else std::cout << "- "; // Ruang kosong
                }
                std::cout << std::endl;
            }
            std::cout << "================\n" << std::endl;
        }

        rclcpp::Publisher<uav_interfaces::msg::MapState>::SharedPtr map_state_pub_;
        uav_interfaces::msg::MapState map_state_message_;
        rclcpp::Subscription<uav_interfaces::msg::DroneStatus>::SharedPtr drone_status_sub_;
        rclcpp::Subscription<uav_interfaces::msg::DroneStatus>::SharedPtr drone_position_sub_;
        rclcpp::Subscription<uav_interfaces::msg::MapState>::SharedPtr path_visualization_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManager>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // Menjalankan callback di beberapa thread

    rclcpp::shutdown();
    return 0;
}