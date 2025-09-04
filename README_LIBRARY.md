# CBS Library for ROS2 Integration

This library provides a C++ interface to the Conflict-Based Search (CBS) algorithm for multi-agent path finding, designed for easy integration with ROS2 nodes.

## Features

- **Simple API**: Easy-to-use interface hiding the complexity of the CBS algorithm
- **Multiple Input Formats**: Support for .pgm files, custom map formats, and programmatic map data
- **Configurable**: Extensive configuration options for different planning scenarios
- **Thread-safe**: Each CBSPlanner instance is independent (but not thread-safe internally)
- **ROS2 Ready**: Designed for seamless ROS2 integration

## Installation

### Prerequisites

- CMake 3.5+
- C++14 compatible compiler
- Boost libraries (program_options, system, filesystem)

### Building the Library

```bash
cd /path/to/CBSH2-RTC
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

This will install:
- Library: `/usr/local/lib/libcbs_lib.so`
- Headers: `/usr/local/include/cbs_lib/`
- CMake config: `/usr/local/lib/cmake/cbs_lib/`

## Usage

### Basic C++ Usage

```cpp
#include <CBSPlanner.h>

int main() {
    cbs_planner::CBSPlanner planner;
    
    // Define agents' start and goal positions
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 4}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}, {4, 0}};
    
    // Configure planning
    cbs_planner::CBSPlanner::Config config;
    config.time_limit = 10.0;
    config.prioritize_conflicts = true;
    
    // Plan paths
    auto result = planner.planPaths("map.pgm", starts, goals, config);
    
    if (result.success) {
        std::cout << "Paths found!" << std::endl;
        std::cout << cbs_planner::CBSPlanner::pathsToString(result.paths);
    }
    
    return 0;
}
```

### ROS2 Integration

#### CMakeLists.txt

```cmake
find_package(cbs_lib REQUIRED)
add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node cbs_lib::cbs_lib)
ament_target_dependencies(my_node rclcpp geometry_msgs nav_msgs)
```

#### Node Implementation

```cpp
#include <rclcpp/rclcpp.hpp>
#include <CBSPlanner.h>

class PlannerNode : public rclcpp::Node {
private:
    std::unique_ptr<cbs_planner::CBSPlanner> planner_;
    
public:
    PlannerNode() : Node("planner_node") {
        planner_ = std::make_unique<cbs_planner::CBSPlanner>();
        // ... setup subscribers, publishers, etc.
    }
    
    void planPaths() {
        // Convert ROS map to CBS format
        std::vector<std::vector<bool>> map_data = convertROSMap();
        
        // Plan paths
        auto result = planner_->planPaths(map_data, starts, goals);
        
        if (result.success) {
            // Publish paths to robot controllers
            publishPaths(result.paths);
        }
    }
};
```

## Configuration Options

The `CBSPlanner::Config` struct provides many options:

- `time_limit`: Maximum planning time in seconds (default: 60.0)
- `node_limit`: Maximum nodes to expand (default: 100000)
- `prioritize_conflicts`: Use conflict prioritization (default: true)
- `bypass`: Use bypass reasoning (default: true)
- `target_reasoning`: Use target reasoning (default: true)
- `mutex_reasoning`: Use mutex reasoning (default: false)
- `use_sipp`: Use SIPP for dynamic environments (default: false)

## Map Formats

### Supported Input Formats

1. **PGM Files**: Binary grayscale images (P5 format)
2. **Grid Maps**: Text-based grid maps
3. **Programmatic**: `std::vector<std::vector<bool>>` where `true` = obstacle

### Map Coordinate System

- Origin (0,0) is at top-left
- First coordinate is row (y-axis)
- Second coordinate is column (x-axis)

## Error Handling

All planning functions return a `Result` object with:
- `success`: Boolean indicating if planning succeeded
- `paths`: Vector of agent paths (if successful)
- `error_message`: Descriptive error message (if failed)
- `runtime`: Planning time in seconds
- `solution_cost`: Total path cost

## Thread Safety

- Each `CBSPlanner` instance is independent
- **Not thread-safe**: Don't call methods on the same instance from multiple threads
- For concurrent planning, create separate `CBSPlanner` instances

## Examples

The `examples/` directory contains comprehensive usage examples with detailed documentation. See `examples/README.md` for complete details.

### 1. Basic Usage (`basic_usage.cpp`)
Demonstrates fundamental library usage including:
- Creating maps programmatically
- Basic configuration options
- Multiple planning scenarios

**Build and run:**
```bash
cd examples
mkdir build && cd build
cmake ..
make
./basic_usage
```

### 2. Advanced Configuration (`advanced_configuration.cpp`)
Shows advanced features and optimization techniques:
- Complex multi-agent scenarios (4+ agents)
- Performance comparison between configurations
- All CBS enhancements (bypass, target reasoning, etc.)
- SIPP for dynamic environments
- Runtime analysis and path inspection

**Build and run:**
```bash
cd examples/build  # (after building basic example)
./advanced_configuration
```

### 3. ROS2 Integration (`ros2_example/`)
Complete ROS2 node demonstrating:
- Service-based planning interface
- Map subscription from map_server
- Path publishing for visualization
- Parameter configuration

**Build and run:**
```bash
cd ros2_example
colcon build
source install/setup.bash
ros2 launch cbs_planner_ros2 cbs_planner.launch.py
```

### Example Output
```bash
=== CBS Library: Advanced Configuration Example ===
Testing 4 agents on a 7x7 complex map...

--- Test 1: Standard CBS ---
✓ Success!
  Solution cost: 48
  Runtime: 0.234 seconds
  Nodes expanded: 1247

--- Test 2: Optimized CBS (all enhancements) ---
✓ Success!
  Solution cost: 46
  Runtime: 0.089 seconds
  Nodes expanded: 423

=== Performance Comparison ===
Standard CBS vs Optimized CBS:
  Node expansion reduction: 66.1%
  Runtime improvement: 62.0%
```

## Performance Tips

1. **Map Size**: Larger maps require more memory and time
2. **Agent Count**: Planning time grows exponentially with agents
3. **Time Limits**: Set reasonable time limits based on your requirements
4. **Configuration**: Enable only necessary reasoning techniques

## Troubleshooting

### Common Issues

1. **"Map file not found"**: Ensure map file path is correct
2. **"Invalid coordinates"**: Check that start/goal positions are within map bounds
3. **"No solution found"**: Try increasing time_limit or node_limit

### Building Issues

1. **Boost not found**: Install boost development packages
2. **CMake version**: Ensure CMake 3.5+ is installed
3. **C++ standard**: Ensure C++14 support

## License

This library maintains the original CBS implementation license. See `license.md` for details.

## Citation

If you use this library in research, please cite:

```
@article{li2019disjoint,
  title={Disjoint splitting for multi-agent path finding with conflict-based search},
  author={Li, Jiaoyang and Ruml, Wheeler and Koenig, Sven},
  journal={Proceedings of the International Conference on Automated Planning and Scheduling},
  volume={29},
  pages={279--283},
  year={2019}
}
```
