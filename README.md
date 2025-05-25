# AMR Path Planning System

A C++ implementation of an Autonomous Mobile Robot (AMR) path planning system using A* algorithm with Catmull-Rom spline smoothing and motor control command generation.

## Features

- **A-star Path Planning**: Efficient pathfinding algorithm with support for 8-directional movement
- **Path Smoothing**: Catmull-Rom spline interpolation for smooth robot trajectories
- **Motor Control**: Differential drive robot command generation with velocity and angular control
- **Visualization**: HTML-based path visualization using Chart.js
- **Obstacle Avoidance**: Grid-based obstacle representation and avoidance

## Project Structure

```
AMR-Path-Planning/
├── src/
│   ├── main.cpp              # Main application entry point
│   ├── path_planning.cpp     # A* algorithm and path smoothing implementation
│   ├── path_planning.h       # Path planning header file
│   ├── motor_control.cpp     # Motor control implementation
│   └── motor_control.h       # Motor control header file
├── build/                    # Build directory (created during compilation)
├── output/
│   └── path_viz.html        # Generated visualization (created at runtime)
├── docs/
│   └── algorithm_details.md # Detailed algorithm documentation
├── CMakeLists.txt           # CMake build configuration
├── Makefile                 # Alternative build system
├── .gitignore               # Git ignore file
├── LICENSE                  # MIT License
└── README.md               # This file
```

## Algorithm Overview

### A* Pathfinding
- Uses Manhattan distance heuristic for grid-based navigation
- Supports diagonal movement with appropriate cost weighting
- Implements cycle detection and iteration limits for robustness

### Path Smoothing
- Catmull-Rom spline interpolation between waypoints
- Configurable sampling rate for trajectory resolution
- Maintains path feasibility while reducing sharp turns

### Motor Commands
- Differential drive kinematics
- Velocity and angular velocity calculation
- Duration-based command generation for real-time control

## Installation

### Prerequisites
- C++11 or later compiler (GCC, Clang, or MSVC)
- CMake 3.10+ (optional, for CMake build)
- Make (for Makefile build)

### Building with CMake
```bash
git clone https://github.com/Puliya07/AMR-Path-Planning.git
cd AMR-Path-Planning
mkdir build && cd build
cmake ..
make
```

### Building with Makefile
```bash
git clone https://github.com/Puliya07/AMR-Path-Planning.git
cd AMR-Path-Planning
make
```

### Manual Compilation
```bash
g++ -std=c++11 -O2 src/*.cpp -o amr_pathfinder
```

## Usage

### Basic Example
```cpp
#include "path_planning.h"
#include "motor_control.h"

// Define grid (0 = free space, 1 = obstacle)
std::vector<std::vector<int>> grid = {
    {0, 0, 0, 1, 0},
    {0, 1, 0, 1, 0},
    {0, 1, 0, 0, 0},
    {0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0}
};

Point start(0, 0);
Point goal(4, 4);

// Find path using A*
std::vector<Point> path = a_star(grid, start, goal);

// Smooth the path
std::vector<PointF> smoothed_path = smooth_path(path, 5);

// Generate motor commands
double robot_speed = 0.5; // m/s
double grid_size = 1.0;   // meters
std::vector<MotorCommand> commands = path_to_motor_commands(
    smoothed_path, robot_speed, grid_size
);
```

### Running the Demo
```bash
./amr_pathfinder
```

This will:
1. Calculate an optimal path through the predefined obstacle grid
2. Apply Catmull-Rom smoothing to the path
3. Generate motor control commands
4. Create an HTML visualization (`path_viz.html`)

### Visualization
Open the generated `path_viz.html` file in a web browser to see:
- Original A* path (red line with circles)
- Smoothed path (blue line with smaller circles)
- Obstacles (black squares)

## Configuration

### Grid Setup
Modify the grid in `main.cpp` to represent your environment:
```cpp
std::vector<std::vector<int>> grid = {
    {0, 0, 1, 0, 0},  // 0 = free space
    {0, 1, 1, 1, 0},  // 1 = obstacle
    {0, 0, 0, 0, 0},
    // ... more rows
};
```

### Robot Parameters
Adjust robot-specific parameters in `path_to_motor_commands()`:
```cpp
const double wheel_base = 0.2;              // meters
const double max_angular_velocity = 2.0;    // rad/s
const double min_distance = 0.01;           // meters
```

### Path Smoothing
Control smoothing resolution in `smooth_path()`:
```cpp
int samples_per_segment = 5; // Higher = smoother but more points
```

## API Reference

### Core Functions

#### `a_star(grid, start, goal)`
- **Parameters**: 2D grid, start point, goal point
- **Returns**: Vector of waypoints representing the optimal path
- **Complexity**: O(V log V) where V is the number of grid cells

#### `smooth_path(path, samples_per_segment)`
- **Parameters**: Original path waypoints, interpolation density
- **Returns**: Smoothed path with floating-point coordinates
- **Algorithm**: Catmull-Rom spline interpolation

#### `path_to_motor_commands(path, speed, grid_size)`
- **Parameters**: Smoothed path, robot speed, physical grid size
- **Returns**: Vector of motor commands with velocities and durations
- **Output**: Differential drive compatible commands

### Data Structures

```cpp
struct Point { int x, y; };                    // Integer grid coordinates
struct PointF { double x, y; };                // Floating-point coordinates
struct MotorCommand {                          // Robot control command
    double linear_velocity;    // m/s
    double angular_velocity;   // rad/s
    double duration;          // seconds
};
```

## Performance

- **Grid Size**: Tested up to 100x100 grids
- **Path Length**: Handles paths with 1000+ waypoints
- **Smoothing**: ~1ms per 10 waypoints on modern hardware
- **Memory Usage**: O(grid_size²) for A* algorithm

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines
- Follow existing code style and naming conventions
- Add unit tests for new functionality
- Update documentation for API changes
- Ensure all tests pass before submitting PR

## Testing

Run the built-in test cases:
```bash
make test  # If using Makefile
# or
ctest     # If using CMake
```

## Troubleshooting

### Common Issues

**No path found**
- Check that start and goal points are not on obstacles
- Verify grid connectivity between start and goal
- Increase iteration limit in A* if needed

**Jerky robot movement**
- Increase smoothing samples per segment
- Adjust maximum angular velocity limits
- Check grid resolution vs. robot size

**Visualization not displaying**
- Ensure Chart.js CDN is accessible
- Check browser console for JavaScript errors
- Verify HTML file generation completed successfully

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- A* algorithm implementation based on Peter Hart, Nils Nilsson, and Bertram Raphael's original paper
- Catmull-Rom spline mathematics from Edwin Catmull and Raphael Rom
- Chart.js library for visualization components

## Citations

```bibtex
@article{hart1968formal,
  title={A formal basis for the heuristic determination of minimum cost paths},
  author={Hart, Peter E and Nilsson, Nils J and Raphael, Bertram},
  journal={IEEE transactions on Systems Science and Cybernetics},
  volume={4},
  number={2},
  pages={100--107},
  year={1968}
}
```

## Contact

- Author: Pulitha Gunawardana
- Email: pulithathiyasara@gmail.com
- Project Link: [https://github.com/Puliya07/AMR-Path-Planning](https://github.com/Puliya07/AMR-Path-Planning)
