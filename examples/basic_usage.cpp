/* Basic example of using the CBS library
 * This example shows how to use the CBSPlanner class for multi-agent path planning
 */

#include <CBSPlanner.h>
#include <iostream>
#include <vector>

int main() {
    // Create a CBS planner instance
    cbs_planner::CBSPlanner planner;
    
    // Example 1: Using a programmatically created map
    std::cout << "=== Example 1: Planning with programmatic map ===" << std::endl;
    
    // Create a simple 5x5 grid map
    // false = free space, true = obstacle
    std::vector<std::vector<bool>> map_data = {
        {false, false, false, false, false},
        {false, true,  true,  false, false},
        {false, false, false, false, false},
        {false, false, true,  true,  false},
        {false, false, false, false, false}
    };
    
    // Define start and goal positions for 2 agents
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {4, 4}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 0}, {0, 4}};
    
    // Configure planning parameters
    cbs_planner::CBSPlanner::Config config;
    config.time_limit = 10.0;              // 10 seconds time limit
    config.prioritize_conflicts = true;     // Use conflict prioritization
    config.bypass = true;                   // Use bypass reasoning
    config.target_reasoning = true;         // Use target reasoning
    
    // Plan paths using the programmatic map
    auto result = planner.planPaths(map_data, starts, goals, config);
    
    if (result.success) {
        std::cout << "Planning successful!" << std::endl;
        std::cout << "Solution cost: " << result.solution_cost << std::endl;
        std::cout << "Runtime: " << result.runtime << " seconds" << std::endl;
        std::cout << "Paths:" << std::endl;
        std::cout << cbs_planner::CBSPlanner::pathsToString(result.paths) << std::endl;
    } else {
        std::cout << "Planning failed: " << result.error_message << std::endl;
    }
    
    // Example 2: Testing different configurations
    std::cout << "\n=== Example 2: Testing different configurations ===" << std::endl;
    
    // Try with SIPP enabled for dynamic environments
    cbs_planner::CBSPlanner::Config sipp_config;
    sipp_config.use_sipp = true;
    sipp_config.time_limit = 5.0;
    
    auto result2 = planner.planPaths(map_data, starts, goals, sipp_config);
    
    if (result2.success) {
        std::cout << "SIPP planning successful!" << std::endl;
        std::cout << "Runtime: " << result2.runtime << " seconds" << std::endl;
    } else {
        std::cout << "SIPP planning failed: " << result2.error_message << std::endl;
    }
    
    return 0;
}
