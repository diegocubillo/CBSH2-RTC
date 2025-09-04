/* Example of using CBS library in a ROS2 node
 * This example shows how to integrate the CBS planner into a ROS2 node
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <CBSPlanner.h>
#include <vector>
#include <memory>

class MultiAgentPlannerNode : public rclcpp::Node
{
public:
    MultiAgentPlannerNode() : Node("multi_agent_planner")
    {
        // Initialize the CBS planner
        planner_ = std::make_unique<cbs_planner::CBSPlanner>();
        
        // Create subscribers and publishers
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&MultiAgentPlannerNode::mapCallback, this, std::placeholders::_1));
            
        // Timer for periodic planning (example)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MultiAgentPlannerNode::planningCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Multi-agent planner node initialized");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Convert ROS occupancy grid to format expected by CBS planner
        current_map_ = convertOccupancyGridToMap(msg);
        map_width_ = msg->info.width;
        map_height_ = msg->info.height;
        map_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Received map of size %dx%d", map_width_, map_height_);
    }
    
    void planningCallback()
    {
        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "No map received yet, skipping planning");
            return;
        }
        
        // Example: Plan paths for 3 agents
        std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {
            {1, 1},   // Agent 0 start
            {1, 5},   // Agent 1 start
            {5, 1}    // Agent 2 start
        };
        
        std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {
            {8, 8},   // Agent 0 goal
            {8, 3},   // Agent 1 goal
            {3, 8}    // Agent 2 goal
        };
        
        // Configure planning parameters
        cbs_planner::CBSPlanner::Config config;
        config.time_limit = 30.0;  // 30 seconds
        config.prioritize_conflicts = true;
        config.bypass = true;
        config.target_reasoning = true;
        
        RCLCPP_INFO(this->get_logger(), "Starting path planning for %zu agents", starts.size());
        
        // Plan paths
        auto result = planner_->planPaths(current_map_, starts, goals, config);
        
        if (result.success) {
            RCLCPP_INFO(this->get_logger(), 
                       "Planning successful! Cost: %d, Runtime: %.3f seconds, Nodes expanded: %d",
                       result.solution_cost, result.runtime, result.num_expanded);
            
            // Print paths
            std::string paths_str = cbs_planner::CBSPlanner::pathsToString(result.paths);
            RCLCPP_INFO(this->get_logger(), "Computed paths:\n%s", paths_str.c_str());
            
            // Here you would typically publish the paths or send them to robot controllers
            // publishPaths(result.paths);
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", result.error_message.c_str());
        }
    }
    
    std::vector<std::vector<bool>> convertOccupancyGridToMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid)
    {
        std::vector<std::vector<bool>> map_data(grid->info.height, std::vector<bool>(grid->info.width));
        
        for (unsigned int i = 0; i < grid->info.height; ++i) {
            for (unsigned int j = 0; j < grid->info.width; ++j) {
                int index = i * grid->info.width + j;
                // Convert occupancy grid values: 
                // -1 (unknown) or > 50 (occupied) -> obstacle
                // <= 50 -> free space
                map_data[i][j] = (grid->data[index] == -1 || grid->data[index] > 50);
            }
        }
        
        return map_data;
    }
    
    // Member variables
    std::unique_ptr<cbs_planner::CBSPlanner> planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<std::vector<bool>> current_map_;
    int map_width_ = 0;
    int map_height_ = 0;
    bool map_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiAgentPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
