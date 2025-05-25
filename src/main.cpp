#include "path_planning.h"
#include "motor_control.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

void generate_html_visualization(const std::vector<Point>& original_path, 
                                const std::vector<PointF>& smoothed_path, 
                                const std::vector<std::vector<int>>& grid) {
    std::ofstream out("output/path_viz.html");
    out << std::fixed << std::setprecision(6);
    out << R"(
<!DOCTYPE html>
<html>
<head>
    <title>AMR Path Visualization</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
    <canvas id="pathChart" width="600" height="600"></canvas>
    <script>
        const ctx = document.getElementById('pathChart').getContext('2d');
        new Chart(ctx, {
            type: 'scatter',
            data: {
                datasets: [
                    {
                        label: 'Original Path',
                        data: [)";
    for (size_t i = 0; i < original_path.size(); ++i) {
        out << "{\"x\": " << original_path[i].x << ", \"y\": " << original_path[i].y << "}";
        if (i < original_path.size() - 1) out << ", ";
    }
    out << R"(],
                        borderColor: '#ff0000',
                        backgroundColor: '#ff0000',
                        showLine: true,
                        pointRadius: 5,
                        pointStyle: 'circle'
                    },
                    {
                        label: 'Smoothed Path',
                        data: [)";
    for (size_t i = 0; i < smoothed_path.size(); ++i) {
        out << "{\"x\": " << smoothed_path[i].x << ", \"y\": " << smoothed_path[i].y << "}";
        if (i < smoothed_path.size() - 1) out << ", ";
    }
    out << R"(],
                        borderColor: '#0000ff',
                        backgroundColor: '#0000ff',
                        showLine: true,
                        pointRadius: 3,
                        pointStyle: 'circle'
                    },
                    {
                        label: 'Obstacles',
                        data: [)";
    bool first = true;
    for (int i = 0; i < grid.size(); ++i) {
        for (int j = 0; j < grid[0].size(); ++j) {
            if (grid[i][j] == 1) {
                if (!first) out << ", ";
                out << "{\"x\": " << i << ", \"y\": " << j << "}";
                first = false;
            }
        }
    }
    out << R"(],
                        borderColor: '#000000',
                        backgroundColor: '#000000',
                        pointStyle: 'rect',
                        pointRadius: 8
                    }
                ]
            },
            options: {
                scales: {
                    x: {
                        type: 'linear',
                        position: 'bottom',
                        min: -1,
                        max: 10,
                        title: { display: true, text: 'X (grid units)' }
                    },
                    y: {
                        type: 'linear',
                        min: -1,
                        max: 10,
                        title: { display: true, text: 'Y (grid units)' }
                    }
                },
                plugins: {
                    title: { display: true, text: 'A* Path vs. Smoothed Path with Obstacles' },
                    legend: { display: true }
                }
            }
        });
    </script>
</body>
</html>
)";
    out.close();
    std::cout << "\nGenerated path_viz.html for visualization\n";
}

int main() {
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0, 1, 1, 1, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 1, 1, 1, 1, 1, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {1, 1, 1, 1, 1, 1, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    Point start(0, 0);
    Point goal(9, 9);

    std::vector<Point> path = a_star(grid, start, goal);
    std::cout << "Original Path (" << path.size() << " points):\n";
    for (const auto& p : path) {
        std::cout << "(" << p.x << ", " << p.y << ")\n";
    }

    std::vector<PointF> smoothed_path = smooth_path(path, 5);
    std::cout << "\nSmoothed Path (" << smoothed_path.size() << " points):\n";
    std::cout << std::fixed << std::setprecision(6);
    for (const auto& p : smoothed_path) {
        std::cout << "(" << p.x << ", " << p.y << ")\n";
    }

    std::vector<std::string> instructions = path_to_instructions(smoothed_path);
    std::cout << "\nMotor Instructions (" << instructions.size() << "):\n";
    for (const auto& instruction : instructions) {
        std::cout << instruction << "\n";
    }

    double robot_speed = 0.5; // m/s
    double grid_size = 1.0; // meters
    std::vector<MotorCommand> commands = path_to_motor_commands(smoothed_path, robot_speed, grid_size);
    std::cout << "\nMotor Commands (" << commands.size() << "):\n";
    MotorController motor;
    for (const auto& command : commands) {
        motor.send_motor_command(command);
    }
    motor.stop();

    generate_html_visualization(path, smoothed_path, grid);

    return 0;
}