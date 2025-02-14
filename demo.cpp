#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <map>
#include <algorithm>  

using namespace std;

struct Node {
    int x, y;
    int g, h;
    Node* parent;
    
    Node(int x, int y, int g, int h, Node* parent = nullptr) : x(x), y(y), g(g), h(h), parent(parent) {}
    
    int f() const { return g + h; }
};

struct Compare {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

vector<pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

vector<pair<int, int>> reconstruct_path(Node* node) {
    vector<pair<int, int>> path;
    while (node) {
        path.push_back({node->x, node->y});
        node = node->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<pair<int, int>> astar(vector<vector<char>>& grid, pair<int, int> start, pair<int, int> goal) {
    priority_queue<Node*, vector<Node*>, Compare> open;
    set<pair<int, int>> closed;
    map<pair<int, int>, Node*> allNodes;
    
    Node* startNode = new Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second));
    open.push(startNode);
    allNodes[start] = startNode;
    
    while (!open.empty()) {
        Node* current = open.top();
        open.pop();
        
        if (current->x == goal.first && current->y == goal.second) {
            vector<pair<int, int>> path = reconstruct_path(current);
            for (auto& entry : allNodes) delete entry.second;
            return path;
        }
        
        closed.insert({current->x, current->y});
        
        for (auto [dx, dy] : directions) {
            int nx = current->x + dx, ny = current->y + dy;
            
            if (nx < 0 || ny < 0 || nx >= grid.size() || ny >= grid[0].size() || grid[nx][ny] == 'o' || closed.count({nx, ny}))
                continue;
            
            int new_g = current->g + 1;
            if (!allNodes.count({nx, ny}) || new_g < allNodes[{nx, ny}]->g) {
                Node* neighbor = new Node(nx, ny, new_g, heuristic(nx, ny, goal.first, goal.second), current);
                open.push(neighbor);
                allNodes[{nx, ny}] = neighbor;
            }
        }
    }
    
    for (auto& entry : allNodes) delete entry.second;
    return {};
}

vector<pair<int, int>> find_optimal_path(vector<vector<char>>& grid, pair<int, int> start, vector<pair<int, int>> targets) {
    vector<pair<int, int>> fullPath;
    pair<int, int> current = start;
    
    while (!targets.empty()) {
        int bestIdx = -1;
        vector<pair<int, int>> bestPath;
        
        for (int i = 0; i < targets.size(); i++) {
            vector<pair<int, int>> path = astar(grid, current, targets[i]);
            if (!path.empty() && (bestPath.empty() || path.size() < bestPath.size())) {
                bestPath = path;
                bestIdx = i;
            }
        }
        
        if (bestIdx == -1) return {};
        
        if (!fullPath.empty()) bestPath.erase(bestPath.begin()); // Avoid duplicate positions
        fullPath.insert(fullPath.end(), bestPath.begin(), bestPath.end());
        current = targets[bestIdx];
        targets.erase(targets.begin() + bestIdx);
    }
    
    vector<pair<int, int>> returnPath = astar(grid, current, start);
    if (!returnPath.empty()) returnPath.erase(returnPath.begin()); // Avoid duplicate positions
    fullPath.insert(fullPath.end(), returnPath.begin(), returnPath.end());
    return fullPath;
}

int main() {
    vector<vector<char>> grid = {
        {'s', '.', '.', 'o', '.'},
        {'.', 'o', '.', 'o', '.'},
        {'.', '.', '.', '.', '.'},
        {'.', 'o', '.', 'o', 't'},
        {'.', '.', '.', '.', 't'}
    };
    
    pair<int, int> start;
    vector<pair<int, int>> targets;
    
    for (int i = 0; i < grid.size(); i++) {
        for (int j = 0; j < grid[0].size(); j++) {
            if (grid[i][j] == 's') start = {i, j};
            else if (grid[i][j] == 't') targets.push_back({i, j});
        }
    }
    
    vector<pair<int, int>> path = find_optimal_path(grid, start, targets);
    
    if (path.empty()) {
        cout << "No valid path found!" << endl;
    } else {
        for (auto [x, y] : path) {
            cout << "(" << x << ", " << y << ") -> ";
        }
        cout << "END" << endl;
    }
    
    return 0;
}
