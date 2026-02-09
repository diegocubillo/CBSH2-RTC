/*
 * test_cbsta_planner.cpp - Integration tests for CBSTAPlanner
 */

#include <gtest/gtest.h>
#include "CBSTAPlanner.h"
#include <chrono>

using namespace cbs_planner;

class CBSTAPlannerTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<CBSTAPlanner>();
        
        // Simple 5x5 map with some obstacles
        simple_5x5 = {
            {false, false, false, false, false},
            {false, true,  true,  false, false},
            {false, false, false, false, false},
            {false, false, true,  true,  false},
            {false, false, false, false, false}
        };
        
        // Open 4x4 map (no obstacles)
        open_4x4 = {
            {false, false, false, false},
            {false, false, false, false},
            {false, false, false, false},
            {false, false, false, false}
        };
    }
    
    std::unique_ptr<CBSTAPlanner> planner;
    std::vector<std::vector<bool>> simple_5x5;
    std::vector<std::vector<bool>> open_4x4;
};

// Test 1: Two agents, two goals, optimal assignment
TEST_F(CBSTAPlannerTest, TwoAgentsTwoGoalsOptimal) {
    std::vector<CBSTAPlanner::Coordinate> starts = {{0, 0}, {0, 3}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{3, 0}, {3, 3}};
    
    // Both agents can reach both goals
    auto result = planner->planPathsWithAssignment(open_4x4, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 2);
    EXPECT_EQ(result.assignment.size(), 2);
    
    // Verify paths start at starts and end at assigned goals
    for (size_t i = 0; i < 2; i++) {
        EXPECT_EQ(result.paths[i].front(), starts[i]);
        int goal_idx = result.assignment[i];
        EXPECT_EQ(result.paths[i].back(), goals[goal_idx]);
    }
    
    // Verify no collisions
    size_t max_time = std::max(result.paths[0].size(), result.paths[1].size());
    for (size_t t = 0; t < max_time; t++) {
        auto pos0 = t < result.paths[0].size() ? result.paths[0][t] : result.paths[0].back();
        auto pos1 = t < result.paths[1].size() ? result.paths[1][t] : result.paths[1].back();
        EXPECT_NE(pos0, pos1) << "Collision at time " << t;
    }
}

// Test 2: Constrained assignment
TEST_F(CBSTAPlannerTest, ConstrainedAssignment) {
    std::vector<CBSTAPlanner::Coordinate> starts = {{0, 0}, {3, 3}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{3, 0}, {0, 3}};
    
    // Agent 0 can only go to goal 0, Agent 1 can only go to goal 1
    std::vector<std::vector<bool>> allowed = {
        {true, false},
        {false, true}
    };
    
    auto result = planner->planPathsWithAssignment(open_4x4, starts, goals, allowed);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.assignment[0], 0);
    EXPECT_EQ(result.assignment[1], 1);
}

// Test 3: More goals than agents
TEST_F(CBSTAPlannerTest, MoreGoalsThanAgents) {
    std::vector<CBSTAPlanner::Coordinate> starts = {{0, 0}, {0, 3}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{3, 0}, {3, 1}, {3, 2}, {3, 3}};
    
    auto result = planner->planPathsWithAssignment(open_4x4, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 2);
    EXPECT_EQ(result.assignment.size(), 2);
    
    // Goals should be different
    EXPECT_NE(result.assignment[0], result.assignment[1]);
}

// Test 4: Swap scenario (agents must coordinate)
TEST_F(CBSTAPlannerTest, SwapScenario) {
    // Agents need to swap positions
    std::vector<CBSTAPlanner::Coordinate> starts = {{1, 0}, {1, 3}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{1, 3}, {1, 0}};
    
    // Each agent can reach either goal
    auto result = planner->planPathsWithAssignment(open_4x4, starts, goals);
    
    EXPECT_TRUE(result.success);
    
    // Solution should find either a swap or optimal assignment
    // If assignment is [1, 0] (swap goals), agents just stay at start = no conflict
    // If assignment is [0, 1] (swap positions), agents need to coordinate
    
    // Verify collision-free
    size_t max_time = std::max(result.paths[0].size(), result.paths[1].size());
    for (size_t t = 0; t < max_time; t++) {
        auto pos0 = t < result.paths[0].size() ? result.paths[0][t] : result.paths[0].back();
        auto pos1 = t < result.paths[1].size() ? result.paths[1][t] : result.paths[1].back();
        EXPECT_NE(pos0, pos1) << "Collision at time " << t;
    }
}

// Test 5: Single agent multiple goals
TEST_F(CBSTAPlannerTest, SingleAgentMultipleGoals) {
    std::vector<CBSTAPlanner::Coordinate> starts = {{0, 0}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{3, 3}, {0, 3}, {3, 0}};  // Different distances
    
    auto result = planner->planPathsWithAssignment(open_4x4, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 1);
    EXPECT_EQ(result.assignment.size(), 1);
    
    // Should pick the closest goal (index 1 or 2, both are distance 3)
    // Goal 0 at (3,3) is distance 6, goals 1 and 2 are distance 3
    EXPECT_NE(result.assignment[0], 0);
}

// Test 6: Three agents, four goals
TEST_F(CBSTAPlannerTest, ThreeAgentsFourGoals) {
    std::vector<CBSTAPlanner::Coordinate> starts = {{0, 0}, {0, 2}, {2, 0}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{2, 2}, {0, 2}, {2, 0}, {0, 0}};
    
    // Open 3x3 map
    std::vector<std::vector<bool>> map_3x3(3, std::vector<bool>(3, false));
    
    auto result = planner->planPathsWithAssignment(map_3x3, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 3);
    
    // All assigned goals should be different
    std::set<int> assigned(result.assignment.begin(), result.assignment.end());
    EXPECT_EQ(assigned.size(), 3);
}

// Test 7: Performance test
TEST_F(CBSTAPlannerTest, PerformanceTest) {
    // Create a 10x10 map
    std::vector<std::vector<bool>> map_10x10(10, std::vector<bool>(10, false));
    // Add some obstacles
    map_10x10[3][3] = map_10x10[3][4] = map_10x10[3][5] = true;
    map_10x10[6][3] = map_10x10[6][4] = map_10x10[6][5] = true;
    
    std::vector<CBSTAPlanner::Coordinate> starts = {{0, 0}, {0, 9}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{9, 9}, {9, 0}, {5, 5}};
    
    CBSTAPlanner::Config config;
    config.time_limit = 10.0;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto result = planner->planPathsWithAssignment(map_10x10, starts, goals, config);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    EXPECT_TRUE(result.success);
    EXPECT_LT(duration.count(), 5000);  // Should complete in < 5 seconds
}

// Test 8: Assignment to string
TEST_F(CBSTAPlannerTest, AssignmentToString) {
    std::vector<CBSTAPlanner::Coordinate> goals = {{0, 0}, {1, 1}, {2, 2}};
    CBSTAPlanner::Assignment assignment = {2, 0, 1};
    
    std::string str = CBSTAPlanner::assignmentToString(assignment, goals);
    
    EXPECT_FALSE(str.empty());
    EXPECT_NE(str.find("Agent 0"), std::string::npos);
    EXPECT_NE(str.find("Goal 2"), std::string::npos);
}

// Test 9: Invalid input handling
TEST_F(CBSTAPlannerTest, InvalidInputHandling) {
    // No agents
    std::vector<CBSTAPlanner::Coordinate> empty_starts;
    std::vector<CBSTAPlanner::Coordinate> goals = {{0, 0}};
    
    auto result = planner->planPathsWithAssignment(open_4x4, empty_starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 10: Fewer goals than agents (should fail)
TEST_F(CBSTAPlannerTest, FewerGoalsThanAgents) {
    std::vector<CBSTAPlanner::Coordinate> starts = {{0, 0}, {0, 1}, {0, 2}};
    std::vector<CBSTAPlanner::Coordinate> goals = {{3, 0}, {3, 1}};  // Only 2 goals for 3 agents
    
    auto result = planner->planPathsWithAssignment(open_4x4, starts, goals);
    
    EXPECT_FALSE(result.success);
}
