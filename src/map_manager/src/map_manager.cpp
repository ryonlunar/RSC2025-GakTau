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

            drone_status_sub_= this->create_subscription<uav_interfaces::msg::DroneStatus>(
                "drone_status", 10, std::bind(&MapManager::drone_status_callback, this, std::placeholders::_1)
            );

            drone_position_sub_= this->create_subscription<uav_interfaces::msg::DroneStatus>(
                "drone_position", 10, std::bind(&MapManager::drone_position_callback, this, std::placeholders::_1)
            );

            //TOPIC map_state
            map_state_pub_ = this->create_publisher<uav_interfaces::msg::MapState>("map_state", 10);
            
            //PARAMS and INITIALIZATION
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

            //TOPIC path_visualization
            path_visualization_sub_ = this->create_subscription<uav_interfaces::msg::MapState>(
                "path_visualization", 10, std::bind(&MapManager::path_visualization_callback, this, std::placeholders::_1)
            );

            //SERVICE /add_victim, /add_obstacle

            add_victim_service_ = this->create_service<uav_interfaces::srv::AddVictim>(
                "/add_victim",
                std::bind(&MapManager::handle_add_victim, this, std::placeholders::_1, std::placeholders::_2)
            );
            
            add_obstacle_service_ = this->create_service<uav_interfaces::srv::AddObstacle>(
                "/add_obstacle",
                std::bind(&MapManager::handle_add_obstacle, this, std::placeholders::_1, std::placeholders::_2)
            );

            obstacle_timer = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MapManager::process_obstacle,this));
            victim_timer = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MapManager::process_victim,this));

            //TOPIC FLAG
            flag_sub_ = this->create_subscription<uav_interfaces::msg::Flag>(
                "flag", 10, std::bind(&MapManager::flag_callback, this, std::placeholders::_1)
            );

            flag_message_.simulation_flag = false;
            flag_message_.found_path_flag = false; 

        }

    private:

        //TOPIC drone_status, drone_position
        rclcpp::Subscription<uav_interfaces::msg::DroneStatus>::SharedPtr drone_status_sub_;
        rclcpp::Subscription<uav_interfaces::msg::DroneStatus>::SharedPtr drone_position_sub_;

        void drone_status_callback(const uav_interfaces::msg::DroneStatus::SharedPtr drone_message) const
        {   
            if (flag_message_.simulation_flag){
                RCLCPP_INFO(this->get_logger(), "SIMULATION");
            }        

            std::cout << "energi = " << drone_message->energy << std::endl;
            std::cout << "penumpang = " << drone_message->passengers << std::endl;
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

        //TOPIC map_state, path_visualization
        rclcpp::Publisher<uav_interfaces::msg::MapState>::SharedPtr map_state_pub_;
        uav_interfaces::msg::MapState map_state_message_;
        rclcpp::Subscription<uav_interfaces::msg::MapState>::SharedPtr path_visualization_sub_;

        void path_visualization_callback(const uav_interfaces::msg::MapState::SharedPtr msg) {
            map_state_message_.width = msg->width;
            map_state_message_.height = msg->height;
            map_state_message_.grid_data = msg->grid_data;
            map_state_message_.path_numbers = msg->path_numbers;
            auto width = map_state_message_.width;
            auto height = map_state_message_.height;

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

        //TOPIC flag
        rclcpp::Subscription<uav_interfaces::msg::Flag>::SharedPtr flag_sub_;
        uav_interfaces::msg::Flag flag_message_;

        void flag_callback(const uav_interfaces::msg::Flag::SharedPtr msg) {
            flag_message_.simulation_flag = msg->simulation_flag;
            flag_message_.found_path_flag = msg->found_path_flag;
        }

        //SERVICE /add_obstacle, /add_victim
        std::priority_queue<obs> obs_queue_;
        std::priority_queue<vic> vic_queue_;
        rclcpp::Service<uav_interfaces::srv::AddVictim>::SharedPtr add_victim_service_;
        rclcpp::Service<uav_interfaces::srv::AddObstacle>::SharedPtr add_obstacle_service_;
        rclcpp::TimerBase::SharedPtr victim_timer;
        rclcpp::TimerBase::SharedPtr obstacle_timer;
        rclcpp::Time start_time_ = this->now();

        void handle_add_victim(
            const std::shared_ptr<uav_interfaces::srv::AddVictim::Request> request,
            std::shared_ptr<uav_interfaces::srv::AddVictim::Response> response) {
            size_t idx = static_cast<size_t>(request->y) * map_state_message_.width + static_cast<size_t>(request->x);

            if (idx >= map_state_message_.grid_data.size()) {
                response->success = false;
                response->message = "Koordinat korban di luar batas peta!";
                return;
            }

            vic_queue_.push(vic(request->x,request->y,request->time));
            response->success = true;
            response->message = "Korban berhasil ditambahkan!";
        }
        
        void process_victim(){
            while( !vic_queue_.empty() ){
                auto &new_vic = vic_queue_.top();
                auto vic_time = start_time_ + rclcpp::Duration::from_seconds(new_vic.time);
                auto now = this->now();

                if( now >= vic_time ){
                    size_t idx = static_cast<size_t>(new_vic.y) * map_state_message_.width + static_cast<size_t>(new_vic.x);
            
                    map_state_message_.grid_data[idx] = 4; // Tanda victim
                    RCLCPP_INFO(this->get_logger(), "Korban ditambahkan di (%d, %d)", new_vic.x, new_vic.y);
            
                    map_state_pub_->publish(map_state_message_);
                    vic_queue_.pop();
                }

                else break;
            }
        }

        void handle_add_obstacle(
            const std::shared_ptr<uav_interfaces::srv::AddObstacle::Request> request,
            std::shared_ptr<uav_interfaces::srv::AddObstacle::Response> response) {
            size_t idx = static_cast<size_t>(request->y) * map_state_message_.width + static_cast<size_t>(request->x);

            if (idx >= map_state_message_.grid_data.size()) {
                response->success = false;
                response->message = "Koordinat obstacle di luar batas peta!";
                return;
            }
            
            obs_queue_.push(obs(request->x,request->y,request->time));
            response->success = true;
            response->message = "Obstacle berhasil ditambahkan!";
        }

        void process_obstacle(){
            while( !obs_queue_.empty() ){
                auto &new_obs = obs_queue_.top();
                auto obs_time = start_time_ + rclcpp::Duration::from_seconds(new_obs.time);
                auto now = this->now();
                if( now >= obs_time ){
                    size_t idx = static_cast<size_t>(new_obs.y) * map_state_message_.width + static_cast<size_t>(new_obs.x);
            
                    map_state_message_.grid_data[idx] = 3; // Tanda obstacle
                    RCLCPP_INFO(this->get_logger(), "Obstacle ditambahkan di (%d, %d)", new_obs.x, new_obs.y);
            
                    map_state_pub_->publish(map_state_message_);
                    obs_queue_.pop();
                }

                else break;
            }
        }

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