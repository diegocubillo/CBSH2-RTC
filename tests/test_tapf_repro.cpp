#include "CBSTAPlanner.h"
#include <gtest/gtest.h>
#include <vector>
#include <iostream>

using namespace cbs_planner;

class CBSTAPlannerReproTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Map from logs
        // 20x20 map
        // . = false (traversable), @ = true (obstacle)
        std::string map_str = 
            "....................\n"
            "....................\n"
            "....................\n"
            "....@@..@@..@@......\n"
            "....@@..@@..@@......\n"
            "....@@..@@..@@......\n"
            "....@@@@@@@@@@@@@...\n"
            "....@@@@@@@@@@@@@...\n"
            "....@@..@@..@@......\n"
            "....@@..@@..@@......\n"
            "....@@@@@@@@@@@@@...\n"
            "....@@@@@@@@@@@@@...\n"
            "....@@..@@..@@......\n"
            "....@@..@@..@@......\n"
            "....@@..@@..@@......\n"
            "....@@..@@..@@......\n"
            "....................\n"
            "....................\n"
            "....................\n"
            "....................";

        parseMap(map_str);
    }

    void parseMap(const std::string& map_str) {
        map_data.clear();
        std::vector<bool> current_row;
        for (char c : map_str) {
            if (c == '\n') {
                if (!current_row.empty()) {
                    map_data.push_back(current_row);
                    current_row.clear();
                }
            } else if (c == '.' || c == '@') {
                current_row.push_back(c == '@');
            }
        }
        if (!current_row.empty()) {
            map_data.push_back(current_row);
        }
    }

    std::vector<std::vector<bool>> map_data;
};

TEST_F(CBSTAPlannerReproTest, ReproduceRosFailure) {
    CBSTAPlanner planner;
    CBSTAPlanner::Config config;
    
    // Config matches ROS node
    // time_limit: 60.0 (default)
    config.time_limit = 5.0; // Shorten for test
    config.screen = 1; // Enable logs
    
    // Coordinates from logs
    // Agent 0: start=(row=19, col=7) -> goal=(row=3, col=19)
    // Agent 1: start=(row=7, col=19) -> goal=(row=19, col=3)
    
    std::vector<CBSTAPlanner::Coordinate> starts;
    starts.emplace_back(19, 7);
    starts.emplace_back(7, 19);
    
    std::vector<CBSTAPlanner::Coordinate> goals;
    goals.emplace_back(3, 19);
    goals.emplace_back(19, 3);
    
    // Allowed matrix (all true)
    // planPathsWithAssignment (without allowed param) creates all-true matrix internally,
    // but the ROS node calls the one WITH allowed param if it constructed it?
    // Let's check cbsh2_rtc_planner.cpp:
    // It calls `cbsta_planner_->planPathsWithAssignment(cbs_map, start_coords, goal_coords, cbsta_config);`
    // So it uses the version WITHOUT allowed matrix, which defaults to all allowed.
    
    auto result = planner.planPathsWithAssignment(map_data, starts, goals, config);
    
    // Debug output
    std::cout << "Result: " << (result.success ? "SUCCESS" : "FAILURE") << std::endl;
    std::cout << "Cost: " << result.solution_cost << std::endl;
    std::cout << "Assignments tried: " << result.num_assignments_tried << std::endl;
    std::cout << "Error: " << result.error_message << std::endl;
    
    if (result.success) {
        std::cout << "Assignment:" << std::endl;
        for (size_t i = 0; i < result.assignment.size(); ++i) {
            std::cout << "Agent " << i << " -> Goal " << result.assignment[i] << std::endl;
        }
    }

    EXPECT_TRUE(result.success) << "Planning should succeed";
    EXPECT_GT(result.solution_cost, 0) << "Solution cost should be positive";
    EXPECT_GE(result.num_assignments_tried, 1) << "Should have tried at least one assignment";
}
