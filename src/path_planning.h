#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H
#include <vector>
#include <string>

struct Point {
    int x, y;
    Point(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

struct PointF {
    double x, y;
    PointF(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
    bool operator==(const PointF& other) const { return x == other.x && y == other.y; }
};

struct Node {
    Point point;
    double f_score;
    Node(Point p, double f) : point(p), f_score(f) {}
    bool operator>(const Node& other) const { return f_score > other.f_score; }
};

struct MotorCommand {
    double linear_velocity;
    double angular_velocity;
    double duration;
    MotorCommand(double lv = 0.0, double av = 0.0, double d = 0.0)
        : linear_velocity(lv), angular_velocity(av), duration(d) {}
};

std::vector<Point> a_star(const std::vector<std::vector<int>>& grid, Point start, Point goal);
std::vector<PointF> smooth_path(const std::vector<Point>& path, int samples_per_segment);
std::vector<std::string> path_to_instructions(const std::vector<PointF>& path);
std::vector<MotorCommand> path_to_motor_commands(const std::vector<PointF>& path, double robot_speed, double grid_size);

#endif