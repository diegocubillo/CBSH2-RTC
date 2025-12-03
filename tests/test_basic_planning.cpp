#include <gtest/gtest.h>
#include <CBSPlanner.h>
#include <chrono>

class BasicPlanningTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
        
        // Create different test maps
        simple_3x3 = {
            {false, false, false},
            {false, true,  false},
            {false, false, false}
        };
        
        simple_5x5 = {
            {false, false, false, false, false},
            {false, true,  true,  false, false},
            {false, false, false, false, false},
            {false, false, true,  true,  false},
            {false, false, false, false, false}
        };
        
        corridor_map = {
            {false, false, false, false, false},
            {false, false, false, false, false}
        };
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
    std::vector<std::vector<bool>> simple_3x3;
    std::vector<std::vector<bool>> simple_5x5;
    std::vector<std::vector<bool>> corridor_map;
};

// Test 1: Single agent simple path (straight line)
TEST_F(BasicPlanningTest, SingleAgentSimplePath) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    auto result = planner->planPaths(simple_3x3, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 1);
    EXPECT_GT(result.paths[0].size(), 0);
    EXPECT_EQ(result.paths[0].front(), starts[0]);
    EXPECT_EQ(result.paths[0].back(), goals[0]);
    EXPECT_GE(result.solution_cost, 4); // Minimum Manhattan distance
}

// Test 2: Single agent with obstacles (must navigate around)
TEST_F(BasicPlanningTest, SingleAgentWithObstacles) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}};
    
    auto result = planner->planPaths(simple_5x5, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 1);
    EXPECT_GT(result.paths[0].size(), 0);
    EXPECT_EQ(result.paths[0].front(), starts[0]);
    EXPECT_EQ(result.paths[0].back(), goals[0]);
    // Path should be at least as long as direct Manhattan distance
    EXPECT_GE(result.solution_cost, 8);
}

// Test 3: Two agents no conflict (independent paths)
TEST_F(BasicPlanningTest, TwoAgentsNoConflict) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 4}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 0}, {4, 4}};
    
    auto result = planner->planPaths(simple_5x5, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 2);
    
    // Check both agents reach their goals
    EXPECT_EQ(result.paths[0].front(), starts[0]);
    EXPECT_EQ(result.paths[0].back(), goals[0]);
    EXPECT_EQ(result.paths[1].front(), starts[1]);
    EXPECT_EQ(result.paths[1].back(), goals[1]);
    
    // Verify no collisions at any timestep
    size_t max_time = std::max(result.paths[0].size(), result.paths[1].size());
    for (size_t t = 0; t < max_time; t++) {
        auto pos0 = t < result.paths[0].size() ? result.paths[0][t] : result.paths[0].back();
        auto pos1 = t < result.paths[1].size() ? result.paths[1][t] : result.paths[1].back();
        EXPECT_NE(pos0, pos1) << "Collision at time " << t;
    }
}

// Test 4: Two agents with conflict (need coordination)
TEST_F(BasicPlanningTest, TwoAgentsWithConflict) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {1, 4}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 4}, {0, 0}};
    
    auto result = planner->planPaths(corridor_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 2);
    
    // Agents should swap positions
    EXPECT_EQ(result.paths[0].front(), starts[0]);
    EXPECT_EQ(result.paths[0].back(), goals[0]);
    EXPECT_EQ(result.paths[1].front(), starts[1]);
    EXPECT_EQ(result.paths[1].back(), goals[1]);
    
    // Solution should be longer than simple paths due to conflict resolution
    EXPECT_GT(result.solution_cost, 8);
}

// Test 5: Multiple agents complex scenario
TEST_F(BasicPlanningTest, MultipleAgentsComplex) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 4}, {4, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}, {4, 0}, {0, 4}};
    
    auto result = planner->planPaths(simple_5x5, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 3);
    
    // All agents should reach their goals
    for (size_t i = 0; i < 3; i++) {
        EXPECT_EQ(result.paths[i].front(), starts[i]);
        EXPECT_EQ(result.paths[i].back(), goals[i]);
        EXPECT_GT(result.paths[i].size(), 0);
    }
}

// Test 6: Agent already at goal
TEST_F(BasicPlanningTest, AgentAtGoal) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 1}};
    
    auto result = planner->planPaths(simple_3x3, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 1);
    EXPECT_EQ(result.paths[0].size(), 1); // Should be just one position
    EXPECT_EQ(result.paths[0][0], starts[0]);
    EXPECT_EQ(result.solution_cost, 0); // No movement needed
}

// Test 7: Multiple agents same start different goals
TEST_F(BasicPlanningTest, SameStartDifferentGoals) {
    // This scenario might not be realistic but tests edge case handling
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{2, 2}, {2, 2}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{0, 0}, {4, 4}};
    
    auto result = planner->planPaths(simple_5x5, starts, goals);
    
    // This should either succeed with coordination or fail gracefully
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 2);
        EXPECT_EQ(result.paths[0].back(), goals[0]);
        EXPECT_EQ(result.paths[1].back(), goals[1]);
    } else {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 8: Path validation helper
TEST_F(BasicPlanningTest, PathValidation) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    auto result = planner->planPaths(simple_3x3, starts, goals);
    
    ASSERT_TRUE(result.success);
    ASSERT_GT(result.paths.size(), 0);
    
    const auto& path = result.paths[0];
    
    // Verify path continuity (adjacent cells)
    for (size_t i = 1; i < path.size(); i++) {
        int dx = std::abs(path[i].first - path[i-1].first);
        int dy = std::abs(path[i].second - path[i-1].second);
        
        // Should move only one cell at a time (Manhattan distance = 1 or 0 for waiting)
        EXPECT_LE(dx + dy, 1) << "Invalid move from (" << path[i-1].first << "," << path[i-1].second 
                              << ") to (" << path[i].first << "," << path[i].second << ")";
    }
    
    // Verify no path goes through obstacles
    for (const auto& pos : path) {
        EXPECT_FALSE(simple_3x3[pos.first][pos.second]) 
            << "Path goes through obstacle at (" << pos.first << "," << pos.second << ")";
    }
}

// Test 9: Path length consistency
TEST_F(BasicPlanningTest, PathLengthConsistency) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 0}};
    
    auto result = planner->planPaths(simple_3x3, starts, goals);
    
    ASSERT_TRUE(result.success);
    
    // For single agent, solution cost should equal path length - 1 (number of moves)
    int expected_cost = result.paths[0].size() - 1;
    EXPECT_EQ(result.solution_cost, expected_cost);
}

// Test 10: Performance on larger path
TEST_F(BasicPlanningTest, PerformanceTest) {
    // Create larger map
    std::vector<std::vector<bool>> large_map(10, std::vector<bool>(10, false));
    // Add some obstacles
    large_map[3][3] = large_map[3][4] = large_map[3][5] = true;
    large_map[6][3] = large_map[6][4] = large_map[6][5] = true;
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{9, 9}};
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto result = planner->planPaths(large_map, starts, goals);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    EXPECT_TRUE(result.success);
    EXPECT_LT(duration.count(), 1000); // Should complete within 1 second
    EXPECT_GT(result.runtime, 0.0);    // Runtime should be measured
}
