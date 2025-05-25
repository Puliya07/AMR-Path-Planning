#define _USE_MATH_DEFINES
#include "path_planning.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>

struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

std::vector<Point> a_star(const std::vector<std::vector<int>>& grid, Point start, Point goal) {
    int rows = grid.size();
    int cols = grid[0].size();

    if (grid[start.x][start.y] == 1 || grid[goal.x][goal.y] == 1) {
        return std::vector<Point>();
    }

    auto heuristic = [](Point a, Point b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };

    std::vector<std::pair<int, int>> neighbors = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_set<Point, PointHash> closed_set;
    open_set.push(Node(start, heuristic(start, goal)));

    std::unordered_map<Point, Point, PointHash> came_from;
    std::unordered_map<Point, double, PointHash> g_score;
    g_score[start] = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (!(i == start.x && j == start.y)) {
                g_score[Point(i, j)] = INFINITY;
            }
        }
    }

    std::unordered_map<Point, double, PointHash> f_score;
    f_score[start] = heuristic(start, goal);

    int iterations = 0;
    const int max_iterations = 10000;
    while (!open_set.empty() && iterations < max_iterations) {
        iterations++;
        Point current = open_set.top().point;
        open_set.pop();

        if (current.x == goal.x && current.y == goal.y) {
            std::vector<Point> path;
            std::unordered_set<Point, PointHash> visited;
            Point curr = goal;
            while (true) {
                if (visited.count(curr)) {
                    return std::vector<Point>();
                }
                visited.insert(curr);
                path.push_back(curr);
                if (curr.x == start.x && curr.y == start.y) break;
                auto it = came_from.find(curr);
                if (it == came_from.end()) {
                    return std::vector<Point>();
                }
                curr = it->second;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed_set.insert(current);
        for (const auto& [dx, dy] : neighbors) {
            Point neighbor(current.x + dx, current.y + dy);
            if (neighbor.x >= 0 && neighbor.x < rows && neighbor.y >= 0 && neighbor.y < cols) {
                if (grid[neighbor.x][neighbor.y] == 0 && !closed_set.count(neighbor)) {
                    double cost = (dx != 0 && dy != 0) ? 1.4 : 1;
                    double tentative_g = g_score[current] + cost;

                    if (tentative_g < g_score[neighbor]) {
                        came_from[neighbor] = current;
                        g_score[neighbor] = tentative_g;
                        f_score[neighbor] = tentative_g + heuristic(neighbor, goal);
                        open_set.push(Node(neighbor, f_score[neighbor]));
                    }
                }
            }
        }
    }
    return std::vector<Point>();
}

std::vector<PointF> smooth_path(const std::vector<Point>& path, int samples_per_segment) {
    if (path.size() < 2) return std::vector<PointF>();

    std::vector<PointF> smoothed_path;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        Point p0 = (i == 0) ? path[0] : path[i - 1];
        Point p1 = path[i];
        Point p2 = path[i + 1];
        Point p3 = (i + 2 < path.size()) ? path[i + 2] : path[i + 1];

        for (int j = 0; j <= samples_per_segment; ++j) {
            double t = static_cast<double>(j) / samples_per_segment;
            double t2 = t * t;
            double t3 = t2 * t;

            double x = 0.5 * (
                (2 * p1.x) +
                (-p0.x + p2.x) * t +
                (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3
            );
            double y = 0.5 * (
                (2 * p1.y) +
                (-p0.y + p2.y) * t +
                (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3
            );

            smoothed_path.emplace_back(x, y); // Store as floating-point
        }
    }
    smoothed_path.push_back(PointF(path.back().x, path.back().y));
    return smoothed_path;
}

std::vector<std::string> path_to_instructions(const std::vector<PointF>& path) {
    std::vector<std::string> instructions;
    if (path.empty()) {
        instructions.push_back("No path found");
        return instructions;
    }

    const double threshold = 0.1; // Minimum movement to consider
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i + 1].x - path[i].x;
        double dy = path[i + 1].y - path[i].y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < threshold) continue;

        double angle = std::atan2(dy, dx) * 180.0 / M_PI;
        std::string direction;

        if (angle >= -22.5 && angle < 22.5) direction = "Move Right";
        else if (angle >= 22.5 && angle < 67.5) direction = "Move Down-Right";
        else if (angle >= 67.5 && angle < 112.5) direction = "Move Down";
        else if (angle >= 112.5 && angle < 157.5) direction = "Move Down-Left";
        else if (angle >= 157.5 || angle < -157.5) direction = "Move Left";
        else if (angle >= -157.5 && angle < -112.5) direction = "Move Up-Left";
        else if (angle >= -112.5 && angle < -67.5) direction = "Move Up";
        else if (angle >= -67.5 && angle < -22.5) direction = "Move Up-Right";
        else direction = "Unknown Move";

        instructions.push_back(direction + " (dist: " + std::to_string(dist) + ")");
    }
    instructions.push_back("Reached Goal");
    return instructions;
}

std::vector<MotorCommand> path_to_motor_commands(const std::vector<PointF>& path, double robot_speed, double grid_size) {
    std::vector<MotorCommand> commands;
    if (path.size() < 2) {
        commands.emplace_back(0.0, 0.0, 0.0);
        return commands;
    }

    const double wheel_base = 0.2;
    const double min_distance = 0.01;
    const double max_angular_velocity = 2.0;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        PointF p1 = path[i];
        PointF p2 = path[i + 1];

        double dx = (p2.x - p1.x) * grid_size;
        double dy = (p2.y - p1.y) * grid_size;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < min_distance) continue;

        double angle = std::atan2(dy, dx);
        double duration = distance / robot_speed;

        double angular_velocity = 0.0;
        if (i > 0) {
            PointF p0 = path[i - 1];
            double prev_dx = (p1.x - p0.x) * grid_size;
            double prev_dy = (p1.y - p0.y) * grid_size;
            double prev_angle = std::atan2(prev_dy, prev_dx);
            double delta_angle = angle - prev_angle;
            while (delta_angle > M_PI) delta_angle -= 2 * M_PI;
            while (delta_angle < -M_PI) delta_angle += 2 * M_PI;
            angular_velocity = std::min(std::max(delta_angle / duration, -max_angular_velocity), max_angular_velocity);
        }

        commands.emplace_back(robot_speed, angular_velocity, duration);
    }
    return commands;
}