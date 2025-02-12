#include <queue>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/map_state.hpp"
#include "uav_interfaces/msg/flag.hpp"

using namespace std::chrono_literals;

struct Node {
    int x, y;
    double g;      // cost-to-come
    double rhs;    // right-hand side value
    std::array<double, 2> key; 
    
    bool operator>(const Node& other) const {
        if (key[0] == other.key[0])
            return key[1] > other.key[1];
        return key[0] > other.key[0];
    }
};

struct Grid {
    std::vector<int> grid_data;  // 1=d, 2=s, 3=o, 4=k
    std::vector<int> path_numbers;
    std::vector<std::vector<double>> heuristic;
    std::unordered_map<int, Node> nodes;
    int width, height;
    int start_x, start_y, goal_x, goal_y;
    double k_m; // accumulative cost change

    Grid(int w, int h) : width(w), height(h), k_m(0) {
        grid_data.resize(w * h, 0);
        path_numbers.resize(w * h, 0);
        heuristic.resize(h, std::vector<double>(w, 0.0));
    }

    int index(int x, int y) const { return y * width + x; }
    
    bool isValidCell(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
};

class FocusedDStar {
private:
    Grid& grid;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    const double INF = std::numeric_limits<double>::infinity();
    
    std::vector<std::pair<int, int>> directions = {{0,1}, {1,0}, {0,-1}, {-1,0}};

    void computeHeuristic(int sx, int sy) {
        for (int y = 0; y < grid.height; ++y) {
            for (int x = 0; x < grid.width; ++x) {
                grid.heuristic[y][x] = std::abs(sx - x) + std::abs(sy - y);
            }
        }
    }

    std::array<double, 2> calculateKey(const Node& node) {
        double min_g_rhs = std::min(node.g, node.rhs);
        return {
            min_g_rhs + grid.heuristic[node.y][node.x] + grid.k_m,
            min_g_rhs
        };
    }

    void updateVertex(Node& node) {
        // menghapus variabel idx yang tidak digunakan
        if (node.g != node.rhs) {
            node.key = calculateKey(node);
            open_list.push(node);
        }
    }

    double calculateRHS(int x, int y) {
        if (x == grid.goal_x && y == grid.goal_y) return 0;
        
        double min_cost = INF;
        for (auto [dx, dy] : directions) {
            int nx = x + dx, ny = y + dy;
            if (!grid.isValidCell(nx, ny)) continue;
            if (grid.grid_data[grid.index(nx, ny)] == 3) continue; // skip obstacles
            
            int nidx = grid.index(nx, ny);
            if (grid.nodes.find(nidx) == grid.nodes.end()) {
                grid.nodes[nidx] = {nx, ny, INF, INF, {0, 0}};
            }
            
            double cost = 1.0; // cost antar-eighbors
            min_cost = std::min(min_cost, grid.nodes[nidx].g + cost);
        }
        return min_cost;
    }

public:
    FocusedDStar(Grid& g) : grid(g) {}

    void initialize() {
        // inisialisasi goal node
        int goal_idx = grid.index(grid.goal_x, grid.goal_y);
        grid.nodes[goal_idx] = {grid.goal_x, grid.goal_y, INF, 0, {0, 0}};
        
        // inisialisasi start node
        int start_idx = grid.index(grid.start_x, grid.start_y);
        grid.nodes[start_idx] = {grid.start_x, grid.start_y, INF, INF, {0, 0}};
        
        // update goal vertex
        Node& goal_node = grid.nodes[goal_idx];
        goal_node.key = calculateKey(goal_node);
        open_list.push(goal_node);
    }

    void computePath() {
        while (!open_list.empty()) {
            Node current = open_list.top();
            int curr_idx = grid.index(current.x, current.y);
            
            if (current.g <= current.rhs && 
                current.x == grid.start_x && 
                current.y == grid.start_y) break;
                
            open_list.pop();
            
            if (current.g > current.rhs) {
                grid.nodes[curr_idx].g = current.rhs;
            } else {
                grid.nodes[curr_idx].g = INF;
                updateVertex(grid.nodes[curr_idx]);
            }
            
            // Update neighbors
            for (auto [dx, dy] : directions) {
                int nx = current.x + dx, ny = current.y + dy;
                if (!grid.isValidCell(nx, ny)) continue;
                
                int nidx = grid.index(nx, ny);
                if (grid.nodes.find(nidx) == grid.nodes.end()) {
                    grid.nodes[nidx] = {nx, ny, INF, INF, {0, 0}};
                }
                
                Node& neighbor = grid.nodes[nidx];
                neighbor.rhs = calculateRHS(nx, ny);
                updateVertex(neighbor);
            }
        }
    }

    void run() {
        computeHeuristic(grid.start_x, grid.start_y);
        initialize();
        computePath();
    }

    void extractPath() {
        int x = grid.start_x, y = grid.start_y;
        int step = 1;
    
        while (x != grid.goal_x || y != grid.goal_y) {
            grid.path_numbers[grid.index(x, y)] = step++;
            
            int best_x = x, best_y = y;
            double min_g = std::numeric_limits<double>::infinity();
    
            // cek neighbors untuk menemukan `g` terkecil
            for (auto [dx, dy] : std::vector<std::pair<int, int>>{{0,1}, {1,0}, {0,-1}, {-1,0}}) {
                int nx = x + dx, ny = y + dy;
                if (!grid.isValidCell(nx, ny)) continue;
    
                int nidx = grid.index(nx, ny);
                if (grid.nodes.find(nidx) != grid.nodes.end() && grid.nodes[nidx].g < min_g) {
                    min_g = grid.nodes[nidx].g;
                    best_x = nx;
                    best_y = ny;
                }
            }
    
            // update posisi ke node dengan g-value terendah
            x = best_x;
            y = best_y;
        }
    
        // tandai posisi goal
        grid.path_numbers[grid.index(grid.goal_x, grid.goal_y)] = step;
    }

    // const std::vector<int> focused_d_star(const uav_interfaces::msg::Flag::SharedPtr msg, Grid& grid, int gx, int gy, int sx, int sy) {
    //     if (msg->simulation_flag){
    //         flag_message_.simulation_flag = true;
    //         grid.goal_x = gx;
    //         grid.goal_y = gy;
    //         grid.start_x = sx;
    //         grid.start_y = sy;
            
    //         FocusedDStar algorithm(grid);
    //         algorithm.run();
    //         algorithm.extractPath();
    //         flag_message_.found_path_flag = true;
    //         flag_pub_->publish(flag_message_);
    //         return grid.path_numbers;
    //     }
    // }
};

class PathPlanner: public rclcpp::Node {
public:
    PathPlanner() : Node("path_planner") {

        //TOPIC map_state, path_visualization
        map_state_sub_ = this->create_subscription<uav_interfaces::msg::MapState>(
            "map_state", 10, std::bind(&PathPlanner::synchronizeGrid, this, std::placeholders::_1)
        );
        path_visualization_pub_ = this->create_publisher<uav_interfaces::msg::MapState>("path_visualization", 10);
        
        //TOPIC flag
        flag_pub_ = this->create_publisher<uav_interfaces::msg::Flag>("flag", 10);
        flag_sub_ = this->create_subscription<uav_interfaces::msg::Flag>(
            "flag", 10, 
            std::bind(&PathPlanner::focused_d_star, this, std::placeholders::_1)
            
        );
        
        //INITIALIZATION flag value
        flag_message_.simulation_flag = false;
        flag_message_.found_path_flag = false; 
    }

private:

    //TOPIC map_state, path_visualization
    rclcpp::Subscription<uav_interfaces::msg::MapState>::SharedPtr map_state_sub_;
    rclcpp::Publisher<uav_interfaces::msg::MapState>::SharedPtr path_visualization_pub_;

    void synchronizeGrid(const uav_interfaces::msg::MapState::SharedPtr msg) {
        grid.grid_data = msg->grid_data;
        grid.width = msg->width;
        grid.height = msg->height;
        // int sx = -1, sy = -1, gx = -1, gy = -1;

        // for (int y = 0; y < grid.height; ++y) {
        //     for (int x = 0; x < grid.width; ++x) {
        //         int idx = grid.index(x, y);
        //         if (grid.grid_data[idx] == 2) { sx = x; sy = y; }
        //         if (grid.grid_data[idx] == 4) { gx = x; gy = y; }
        //     }
        // }

        publishPath(); // publish path (menampilkan dan update map tiap 0.5 detik)
    }

    void publishPath() {
        auto message = uav_interfaces::msg::MapState();
        message.grid_data = grid.grid_data;
        message.path_numbers = grid.path_numbers;
        message.width = grid.width;
        message.height = grid.height;
        path_visualization_pub_->publish(message);

        if (flag_message_.simulation_flag) {
            RCLCPP_INFO(this->get_logger(), "Updating path [SIMULATION]");
        }

    }

    //TOPIC flag
    rclcpp::Publisher<uav_interfaces::msg::Flag>::SharedPtr flag_pub_;
    rclcpp::Subscription<uav_interfaces::msg::Flag>::SharedPtr flag_sub_;
    uav_interfaces::msg::Flag flag_message_;

    void focused_d_star(const uav_interfaces::msg::Flag::SharedPtr msg) {
        if(msg->simulation_flag && !msg->found_path_flag){
            flag_message_.simulation_flag = msg->simulation_flag;
            // lakukan algoritma focused d*
            // jika sudah
            flag_message_.found_path_flag = true;
            flag_pub_->publish(flag_message_);
        } 
    }

    //GRID (struktur data untuk map)
    Grid grid = Grid(7, 7); // Contoh ukuran grid
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
