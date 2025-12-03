# CBS Library Examples

This directory contains practical examples demonstrating how to use the CBS library for multi-agent path finding.

## Available Examples

### 1. Basic Usage (`basic_usage.cpp`)
**Purpose**: Introduction to core CBS library functionality

**Demonstrates**:
- Creating maps programmatically
- Basic agent configuration
- Simple path planning scenarios
- Different planning configurations (standard vs SIPP)

**Output**: Shows successful path planning with solution cost and runtime information.

### 2. Advanced Configuration (`advanced_configuration.cpp`)
**Purpose**: Performance optimization and advanced features

**Demonstrates**:
- Complex multi-agent scenarios (4+ agents)
- Performance comparison between configurations
- All CBS enhancement techniques:
  - Conflict prioritization
  - Bypass reasoning
  - Target reasoning
  - Corridor reasoning
  - Rectangle reasoning
  - Mutex reasoning
- SIPP for dynamic environments
- Runtime analysis and path inspection

**Output**: Detailed performance metrics and comparisons between different CBS configurations.

### 3. ROS2 Integration (`ros2_example/`)
**Purpose**: Integration with ROS2 robotics framework

**Demonstrates**:
- ROS2 node implementation using CBS library
- Service-based path planning interface
- ROS2 message handling for multi-agent scenarios
- Integration with ROS2 navigation stack

**Features**:
- Subscribes to `/map` topic (nav_msgs/OccupancyGrid)
- Plans paths for 3 agents with predefined coordinates every 5 seconds
- Outputs solution paths to console/logs
- Compatible with ROS2 navigation stack map publishers

## Building and Running

### Prerequisites
- CBS library must be built and installed first:
  ```bash
  cd /path/to/CBSH2-RTC
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  ```
- CMake 3.5+
- C++14 compatible compiler
- For ROS2 example: ROS2 Humble or later

### Build Instructions
```bash
# From the examples directory
mkdir build
cd build
cmake ..
make
```

### Run Examples
```bash
# Basic usage example
./basic_usage

# Advanced configuration example
./advanced_configuration
```

### ROS2 Example Setup and Testing

#### 1. Copy to ROS2 Workspace
```bash
# Copy the ROS2 example to your ROS2 workspace
cp -r ros2_example ~/ros2_ws/src/my_ros2_planner_node
cd ~/ros2_ws
```

#### 2. Build the ROS2 Package
```bash
# Build the package
colcon build --packages-select my_ros2_planner_node

# Source the workspace
source install/setup.bash
```

#### 3. Run the ROS2 Node
```bash
# Start the planner node
ros2 run my_ros2_planner_node planner_node
```

#### 4. Publish a Map
The node expects a map from the `/map` topic. You can either:

**Option A**: Use an existing map publisher
```bash
# If you have a map server running
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml
```

**Option B**: Publish a simple test map
```bash
# Publish a test occupancy grid (requires custom script or existing map data)
ros2 topic pub /map nav_msgs/msg/OccupancyGrid '{header: {frame_id: "map"}, info: {resolution: 0.1, width: 10, height: 10}, data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}'
```

#### 5. Monitor Node Activity
```bash
# Check if the node is running
ros2 node list

# Monitor node logs (you'll see the planning output here)
ros2 run rqt_console rqt_console

# Or view logs directly
ros2 log info /multi_agent_planner
```

The node will automatically plan paths every 5 seconds for these agents:
- **Agent 0**: Start (1,1) → Goal (8,8)  
- **Agent 1**: Start (1,5) → Goal (8,3)
- **Agent 2**: Start (5,1) → Goal (3,8)

## Example Output

### Basic Usage
```
=== Example 1: Planning with programmatic map ===
Planning successful!
Solution cost: 8
Runtime: 3.6e-05 seconds
Paths:
Agent 0: (0,0)->(1,0)->(2,0)->(3,0)->(4,0)
Agent 1: (4,4)->(3,4)->(2,4)->(1,4)->(0,4)

=== Example 2: Testing different configurations ===
SIPP planning successful!
Runtime: 2.6e-05 seconds
```

### Advanced Configuration
```
=== CBS Library: Advanced Configuration Example ===
Testing 4 agents on a 7x7 complex map...

--- Test 1: Standard CBS ---
✓ Success!
  Solution cost: 48
  Runtime: 0.000149 seconds
  Nodes expanded: 1

--- Test 2: Optimized CBS (all enhancements) ---
✓ Success!
  Solution cost: 48
  Runtime: 5.8e-05 seconds
  Nodes expanded: 0

=== Performance Comparison ===
Standard CBS vs Optimized CBS:
  Node expansion reduction: 100%
  Runtime improvement: 61.1%
```

## Understanding the Output

- **Solution cost**: Total cost of all agent paths combined
- **Runtime**: Time spent by the CBS algorithm (in seconds)
- **Nodes expanded**: Number of CBS tree nodes explored (lower is better)
- **Wall-clock time**: Actual execution time including overhead

## Customization

You can modify these examples to:
- Test different map sizes and configurations
- Experiment with various agent counts
- Try different planning parameters
- Implement custom performance metrics

## Next Steps

- Check out the `ros2_example/` directory for ROS2 integration details
- For production ROS2 usage, customize the service interface in `planner_node.cpp`
- Integrate with ROS2 navigation stack by subscribing to map and goal topics
- Read `../README_LIBRARY.md` for complete API documentation
- Run the unit tests with `make cbs_tests` in the `../build/` directory

## ROS2 Integration Notes

The ROS2 example provides a foundation for integrating CBS planning into robotic systems:

- **Map Subscription**: The node subscribes to `/map` topic to receive occupancy grids
- **Automatic Planning**: Performs path planning every 5 seconds with predefined agent positions
- **Console Output**: Prints computed paths and planning statistics to the terminal
- **Easy Customization**: Modify start/goal positions and planning frequency in the source code

**Expected Output**:
```
[INFO] [multi_agent_planner]: Multi-agent planner node initialized
[INFO] [multi_agent_planner]: Received map of size 10x10
[INFO] [multi_agent_planner]: Starting path planning for 3 agents
[INFO] [multi_agent_planner]: Planning successful! Cost: 24, Runtime: 0.003 seconds, Nodes expanded: 5
[INFO] [multi_agent_planner]: Computed paths:
Agent 0: (1,1)->(2,1)->(3,1)->(4,1)->(5,1)->(6,1)->(7,1)->(8,1)->(8,2)->(8,3)...
Agent 1: (1,5)->(2,5)->(3,5)->(4,5)->(5,5)->(6,5)->(7,5)->(8,5)->(8,4)->(8,3)
Agent 2: (5,1)->(5,2)->(5,3)->(5,4)->(5,5)->(4,5)->(3,5)->(3,6)->(3,7)->(3,8)
```

For production use, consider:
- Adding dynamic start/goal position subscribers
- Implementing path visualization publishers for RViz
- Adding parameter server configuration for planning settings
- Integrating with robot controllers for path execution
