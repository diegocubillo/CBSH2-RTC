/* Advanced configuration example for the CBS library
 * This example demonstrates advanced planning configurations,
 * performance analysis, and optimization techniques
 */

#include <CBSPlanner.h>
#include <iostream>
#include <vector>
#include <chrono>

int main() {
    std::cout << "=== CBS Library: Advanced Configuration Example ===" << std::endl;
    
    // Create a CBS planner instance
    cbs_planner::CBSPlanner planner;
    
    // Create a more complex 7x7 grid map with multiple obstacles
    std::vector<std::vector<bool>> complex_map = {
        {false, false, false, false, false, false, false},
        {false, true,  true,  false, true,  true,  false},
        {false, false, false, false, false, false, false},
        {true,  false, true,  false, true,  false, true },
        {false, false, false, false, false, false, false},
        {false, true,  true,  false, true,  true,  false},
        {false, false, false, false, false, false, false}
    };
    
    // Define start and goal positions for 4 agents (more challenging scenario)
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {
        {0, 0},   // Agent 0: top-left
        {0, 6},   // Agent 1: top-right
        {6, 0},   // Agent 2: bottom-left
        {6, 6}    // Agent 3: bottom-right
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {
        {6, 6},   // Agent 0: to bottom-right
        {6, 0},   // Agent 1: to bottom-left
        {0, 6},   // Agent 2: to top-right
        {0, 0}    // Agent 3: to top-left
    };
    
    std::cout << "Testing " << starts.size() << " agents on a 7x7 complex map..." << std::endl;
    
    // Test 1: Standard CBS configuration
    std::cout << "\n--- Test 1: Standard CBS ---" << std::endl;
    cbs_planner::CBSPlanner::Config standard_config;
    standard_config.time_limit = 30.0;
    standard_config.prioritize_conflicts = false;
    standard_config.bypass = false;
    standard_config.target_reasoning = false;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto result1 = planner.planPaths(complex_map, starts, goals, standard_config);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (result1.success) {
        std::cout << "✓ Success!" << std::endl;
        std::cout << "  Solution cost: " << result1.solution_cost << std::endl;
        std::cout << "  Runtime: " << result1.runtime << " seconds" << std::endl;
        std::cout << "  Nodes expanded: " << result1.num_expanded << std::endl;
        std::cout << "  Wall-clock time: " << duration1.count() << " ms" << std::endl;
    } else {
        std::cout << "✗ Failed: " << result1.error_message << std::endl;
    }
    
    // Test 2: Optimized CBS with all enhancements
    std::cout << "\n--- Test 2: Optimized CBS (all enhancements) ---" << std::endl;
    cbs_planner::CBSPlanner::Config optimized_config;
    optimized_config.time_limit = 30.0;
    optimized_config.prioritize_conflicts = true;    // Enable conflict prioritization
    optimized_config.bypass = true;                  // Enable bypass reasoning
    optimized_config.target_reasoning = true;        // Enable target reasoning
    optimized_config.corridor_reasoning = true;      // Enable corridor reasoning
    optimized_config.rectangle_reasoning = true;     // Enable rectangle reasoning
    optimized_config.mutex_reasoning = true;         // Enable mutex reasoning
    
    start_time = std::chrono::high_resolution_clock::now();
    auto result2 = planner.planPaths(complex_map, starts, goals, optimized_config);
    end_time = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (result2.success) {
        std::cout << "✓ Success!" << std::endl;
        std::cout << "  Solution cost: " << result2.solution_cost << std::endl;
        std::cout << "  Runtime: " << result2.runtime << " seconds" << std::endl;
        std::cout << "  Nodes expanded: " << result2.num_expanded << std::endl;
        std::cout << "  Wall-clock time: " << duration2.count() << " ms" << std::endl;
    } else {
        std::cout << "✗ Failed: " << result2.error_message << std::endl;
    }
    
    // Test 3: SIPP configuration for dynamic environments
    std::cout << "\n--- Test 3: SIPP (for dynamic environments) ---" << std::endl;
    cbs_planner::CBSPlanner::Config sipp_config;
    sipp_config.use_sipp = true;
    sipp_config.time_limit = 30.0;
    sipp_config.prioritize_conflicts = true;
    
    start_time = std::chrono::high_resolution_clock::now();
    auto result3 = planner.planPaths(complex_map, starts, goals, sipp_config);
    end_time = std::chrono::high_resolution_clock::now();
    auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (result3.success) {
        std::cout << "✓ Success!" << std::endl;
        std::cout << "  Solution cost: " << result3.solution_cost << std::endl;
        std::cout << "  Runtime: " << result3.runtime << " seconds" << std::endl;
        std::cout << "  Nodes expanded: " << result3.num_expanded << std::endl;
        std::cout << "  Wall-clock time: " << duration3.count() << " ms" << std::endl;
    } else {
        std::cout << "✗ Failed: " << result3.error_message << std::endl;
    }
    
    // Performance comparison
    if (result1.success && result2.success) {
        std::cout << "\n=== Performance Comparison ===" << std::endl;
        std::cout << "Standard CBS vs Optimized CBS:" << std::endl;
        std::cout << "  Node expansion reduction: " 
                  << ((double)(result1.num_expanded - result2.num_expanded) / result1.num_expanded * 100.0)
                  << "%" << std::endl;
        std::cout << "  Runtime improvement: " 
                  << ((result1.runtime - result2.runtime) / result1.runtime * 100.0)
                  << "%" << std::endl;
    }
    
    // Example of path analysis
    if (result2.success) {
        std::cout << "\n=== Path Analysis ===" << std::endl;
        std::cout << "Individual agent path lengths:" << std::endl;
        for (size_t i = 0; i < result2.paths.size(); ++i) {
            std::cout << "  Agent " << i << ": " << result2.paths[i].size() << " steps" << std::endl;
        }
        
        std::cout << "\nFirst few steps for Agent 0:" << std::endl;
        const auto& path0 = result2.paths[0];
        for (size_t t = 0; t < std::min(size_t(5), path0.size()); ++t) {
            std::cout << "  Step " << t << ": (" << path0[t].first << ", " << path0[t].second << ")" << std::endl;
        }
    }
    
    return 0;
}
