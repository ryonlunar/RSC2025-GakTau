#include <memory>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uav_interfaces/msg/drone_status.hpp"
#include "uav_interfaces/msg/map_state.hpp"
#include "uav_interfaces/action/rescue_mission.hpp"
#include "uav_interfaces/msg/flag.hpp"

struct MyNode {
    int x, y;
    double g, rhs;
    bool operator>(const MyNode& other) const { return g > other.g; }
};

class PathPlanner : public rclcpp::Node {
public:
    using RescueMission = uav_interfaces::action::RescueMission;
    using GoalHandleRescueMission = rclcpp_action::ClientGoalHandle<RescueMission>;

    PathPlanner() : Node("path_planner") {
        drone_position_sub_ = this->create_subscription<uav_interfaces::msg::DroneStatus>(
            "drone_position", 10, std::bind(&PathPlanner::drone_position_callback, this, std::placeholders::_1));
        
        map_state_sub_ = this->create_subscription<uav_interfaces::msg::MapState>(
            "map_state", 10, std::bind(&PathPlanner::map_state_callback, this, std::placeholders::_1));
        
        rescue_goal_sub_ = this->create_subscription<uav_interfaces::action::RescueMission::Goal>(
            "rescue_goal", 10, std::bind(&PathPlanner::goal_callback, this, std::placeholders::_1));
        
        flag_sub_= this->create_subscription<uav_interfaces::msg::Flag>(
            "flag", 10, std::bind(&PathPlanner::flag_callback, this, std::placeholders::_1));

        flag_pub_ = this->create_publisher<uav_interfaces::msg::Flag>("flag", 10);
            
        path_pub_ = this->create_publisher<uav_interfaces::msg::MapState>("path_visualization", 10);
    }

private:
    rclcpp::Subscription<uav_interfaces::msg::DroneStatus>::SharedPtr drone_position_sub_;
    rclcpp::Subscription<uav_interfaces::msg::MapState>::SharedPtr map_state_sub_;
    rclcpp::Subscription<uav_interfaces::action::RescueMission::Goal>::SharedPtr rescue_goal_sub_;
    rclcpp::Subscription<uav_interfaces::msg::Flag>::SharedPtr flag_sub_;
    rclcpp::Publisher<uav_interfaces::msg::Flag>::SharedPtr flag_pub_;
    rclcpp::Subscription<uav_interfaces::msg::MapState>::SharedPtr path_sub_;
    rclcpp::Publisher<uav_interfaces::msg::MapState>::SharedPtr path_pub_;

    uav_interfaces::msg::DroneStatus current_drone_position_;
    uav_interfaces::msg::MapState current_map_state_;
    RescueMission::Goal current_goal_;

    std::priority_queue<MyNode, std::vector<MyNode>, std::greater<MyNode>> open_list_;
    std::unordered_map<int, std::unordered_map<int, MyNode>> nodes_;

    bool start_flag_ = false;

    void drone_position_callback(const uav_interfaces::msg::DroneStatus::SharedPtr msg) {
        current_drone_position_ = msg;
        plan_path();
    }

    void map_state_callback(const uav_interfaces::msg::MapState::SharedPtr msg) {
        current_map_state_ = msg;
        if( !start_flag_) {
            path_pub_->publish(msg); 
        } else {
            plan_path();
        }
    }

    void goal_callback(const uav_interfaces::action::RescueMission::Goal::SharedPtr msg) {
        current_goal_ = msg;
        RCLCPP_INFO(this->get_logger(), "Goal diterima: (%d, %d)", 
                    current_goal_.target_x, current_goal_.target_y);
    }

    void flag_callback(const uav_interfaces::msg::Flag::SharedPtr msg) {
        start_flag_ = msg->start_flag;
    }
    
    void plan_path() {
        if (!start_flag_) return; 
        RCLCPP_INFO(this->get_logger(), "Menggunakan Focused D* untuk mencari jalur dari (%d, %d) ke (%d, %d)", 
            current_drone_position_.position.x, current_drone_position_.position.y, 
            current_goal_.target_x, current_goal_.target_y);
        
        MyNode start{current_drone_position_.position.x, current_drone_position_.position.y, 0, 0};
        MyNode goal{current_goal_.target_x, current_goal_.target_y, INFINITY, 0};
        
        nodes_[goal.x][goal.y] = goal;
        nodes_[start.x][start.y] = start;
        open_list_.push(goal);
        
        while (!open_list_.empty()) {
            MyNode current = open_list_.top();
            open_list_.pop();
            
            if (current.x == start.x && current.y == start.y) break;
            
            for (const auto& [dx, dy] : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
                int nx = current.x + dx;
                int ny = current.y + dy;
                if (nx < 0 || ny < 0 || nx >= current_map_state_.width || ny >= current_map_state_.height) continue;
                
                if (current_map_state_.grid_data[nx + ny * current_map_state_.width] == 3) continue;
                
                double new_cost = current.g + 1;
                if (nodes_[nx][ny].g > new_cost) {
                    nodes_[nx][ny] = {nx, ny, new_cost, new_cost};
                    open_list_.push(nodes_[nx][ny]);
                }
            }
        }
        
        uav_interfaces::msg::MapState planned_path;
        MyNode current = nodes_[start.x][start.y];
        while (!(current.x == goal.x && current.y == goal.y)) {
            planned_path.path_numbers.push_back(current.x + current.y * current_map_state_.width);
            current = nodes_[current.x + 1][current.y];
            std::cout << current.x << " " << current.y << std::endl;
        }
        planned_path.path_numbers.push_back(goal.x + goal.y * current_map_state_.width);
        planned_path.grid_data = current_map_state_.grid_data;
        planned_path.width = current_map_state_.width;
        planned_path.height = current_map_state_.height;
        path_pub_->publish(planned_path);

        uav_interfaces::msg::Flag flag_message;
        flag_message.start_flag = false;
        flag_message.found_path_flag = true;
        flag_message.simulation_flag = true;
        flag_pub_->publish(flag_message);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
