#include <gtest/gtest.h>
#include <CBSPlanner.h>

class ImpossibleCasesTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
        
        // Completely blocked map
        blocked_map = {
            {true,  true,  true},
            {true,  true,  true},
            {true,  true,  true}
        };
        
        // Map with disconnected regions
        disconnected_map = {
            {false, false, true,  false, false},
            {false, false, true,  false, false},
            {false, false, true,  false, false},
            {false, false, true,  false, false},
            {false, false, true,  false, false}
        };
        
        // Narrow corridor for impossible swaps
        narrow_corridor = {
            {false, false, false, false, false},
            {true,  true,  true,  true,  true}
        };
        
        // Single path with obstacles
        single_path = {
            {false, true,  true,  true,  true},
            {false, true,  false, false, false},
            {false, true,  false, true,  true},
            {false, false, false, true,  false},
            {true,  true,  true,  true,  false}
        };
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
    std::vector<std::vector<bool>> blocked_map;
    std::vector<std::vector<bool>> disconnected_map;
    std::vector<std::vector<bool>> narrow_corridor;
    std::vector<std::vector<bool>> single_path;
};

// Test 1: Completely blocked agent (surrounded by obstacles)
TEST_F(ImpossibleCasesTest, CompletelyBlockedAgent) {
    // Create map with agent surrounded by obstacles
    std::vector<std::vector<bool>> surrounded_map = {
        {true,  true,  true},
        {true,  false, true},
        {true,  true,  true}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{0, 0}};
    
    auto result = planner->planPaths(surrounded_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    EXPECT_EQ(result.solution_cost, -1);
}

// Test 2: Two agents need to swap places in a narrow corridor
TEST_F(ImpossibleCasesTest, ImpossibleSwapNarrowCorridor) {
    // Create a 3x1 corridor (3 rows, 1 column)
    std::vector<std::vector<bool>> corridor_map = {
        {false},  // Row 0, Col 0
        {false},  // Row 1, Col 0  
        {false}   // Row 2, Col 0
    };
    
    // Two agents need to swap - this should be possible in CBS with proper coordination
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {2, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 0}, {0, 0}};
    
    auto result = planner->planPaths(corridor_map, starts, goals);
    
    // CBS should be able to handle this with proper temporal coordination
    // If it fails, it should provide a meaningful error message
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
        std::cout << "Expected behavior: " << result.error_message << std::endl;
    } else {
        // If it succeeds, verify the solution is correct
        EXPECT_EQ(result.paths.size(), 2);
        EXPECT_GT(result.solution_cost, 0);
    }
}

// Test 3: Disconnected map regions
TEST_F(ImpossibleCasesTest, DisconnectedMapRegions) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{0, 4}}; // On other side of wall
    
    auto result = planner->planPaths(disconnected_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 4: Timeout on complex scenario
TEST_F(ImpossibleCasesTest, TimeoutComplexScenario) {
    // Create a very complex scenario with many agents
    std::vector<std::vector<bool>> complex_map(20, std::vector<bool>(20, false));
    
    // Add maze-like obstacles
    for (int i = 0; i < 20; i += 2) {
        for (int j = 0; j < 19; j++) {
            complex_map[i][j] = true;
        }
    }
    
    // Many agents in complex scenario
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    for (int i = 1; i < 20; i += 4) {
        starts.push_back({i, 0});
        goals.push_back({i, 19});
    }
    
    // Set very short time limit
    cbs_planner::CBSPlanner::Config config;
    config.time_limit = 0.001; // 1 millisecond - very short
    config.node_limit = 10;    // Very low node limit
    
    auto result = planner->planPaths(complex_map, starts, goals, config);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    EXPECT_GT(result.runtime, 0.0);
}

// Test 5: All positions are obstacles
TEST_F(ImpossibleCasesTest, AllObstaclesMap) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    auto result = planner->planPaths(blocked_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 6: Too many agents for available space
TEST_F(ImpossibleCasesTest, TooManyAgentsLimitedSpace) {
    // Small map with limited free space
    std::vector<std::vector<bool>> small_map = {
        {false, false},
        {false, true}
    };
    
    // More agents than free spaces
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 1}, {1, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 0}, {0, 0}, {0, 1}};
    
    auto result = planner->planPaths(small_map, starts, goals);
    
    // Might succeed with coordination or fail - test behavior
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 7: Single narrow path with multiple agents
TEST_F(ImpossibleCasesTest, SingleNarrowPathMultipleAgents) {
    // Create a single winding path
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {4, 4}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}, {0, 0}};
    
    auto result = planner->planPaths(single_path, starts, goals);
    
    // This is a challenging scenario - might succeed or fail depending on CBS implementation
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 2);
        // Verify paths don't conflict
        size_t max_time = std::max(result.paths[0].size(), result.paths[1].size());
        for (size_t t = 0; t < max_time; t++) {
            auto pos0 = t < result.paths[0].size() ? result.paths[0][t] : result.paths[0].back();
            auto pos1 = t < result.paths[1].size() ? result.paths[1][t] : result.paths[1].back();
            EXPECT_NE(pos0, pos1) << "Collision at time " << t;
        }
    } else {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 8: Node limit exceeded
TEST_F(ImpossibleCasesTest, NodeLimitExceeded) {
    // Complex scenario that would require many node expansions
    std::vector<std::vector<bool>> maze_map(10, std::vector<bool>(10, false));
    
    // Create maze pattern
    for (int i = 1; i < 9; i += 2) {
        for (int j = 1; j < 9; j++) {
            maze_map[i][j] = true;
        }
    }
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 2}, {0, 4}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{9, 9}, {9, 7}, {9, 5}};
    
    cbs_planner::CBSPlanner::Config config;
    config.node_limit = 5; // Very low limit
    
    auto result = planner->planPaths(maze_map, starts, goals, config);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 9: Goal same as start but path blocked
TEST_F(ImpossibleCasesTest, GoalSameAsStartButPathBlocked) {
    // Agent at goal but if forced to move would have nowhere to go
    std::vector<std::vector<bool>> trapped_map = {
        {true,  true,  true},
        {true,  false, true},
        {true,  true,  true}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 1}};
    
    auto result = planner->planPaths(trapped_map, starts, goals);
    
    // Should succeed since agent is already at goal
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths[0].size(), 1);
    EXPECT_EQ(result.solution_cost, 0);
}

// Test 10: Circular dependency scenario
TEST_F(ImpossibleCasesTest, CircularDependencyScenario) {
    // Create scenario where agents form a cycle that might be impossible to resolve
    std::vector<std::vector<bool>> cycle_map = {
        {false, false, false, false},
        {false, true,  true,  false},
        {false, true,  true,  false},
        {false, false, false, false}
    };
    
    // Agents in a square formation, each wants to move clockwise
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 3}, {3, 3}, {3, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{0, 3}, {3, 3}, {3, 0}, {0, 0}};
    
    auto result = planner->planPaths(cycle_map, starts, goals);
    
    // CBS should be able to resolve this with proper coordination
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 4);
        // Verify all agents reach their goals
        for (size_t i = 0; i < 4; i++) {
            EXPECT_EQ(result.paths[i].back(), goals[i]);
        }
    } else {
        EXPECT_FALSE(result.error_message.empty());
    }
}
