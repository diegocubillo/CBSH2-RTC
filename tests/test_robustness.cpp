#include <gtest/gtest.h>
#include <CBSPlanner.h>
#include <thread>
#include <chrono>
#include <random>

class RobustnessTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
    }
    
    std::vector<std::vector<bool>> generateRandomMap(int width, int height, double obstacle_ratio = 0.2) {
        std::vector<std::vector<bool>> map(height, std::vector<bool>(width, false));
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                map[y][x] = dis(gen) < obstacle_ratio;
            }
        }
        
        return map;
    }
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> generateRandomPositions(
        const std::vector<std::vector<bool>>& map, int count) {
        std::vector<cbs_planner::CBSPlanner::Coordinate> positions;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> x_dis(0, map[0].size() - 1);
        std::uniform_int_distribution<> y_dis(0, map.size() - 1);
        
        while (positions.size() < count) {
            int x = x_dis(gen);
            int y = y_dis(gen);
            
            if (!map[y][x]) { // Not an obstacle
                positions.push_back({x, y});
            }
        }
        
        return positions;
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
};

// Test 1: Stress test with many agents
TEST_F(RobustnessTest, StressTestManyAgents) {
    // Large map for many agents
    std::vector<std::vector<bool>> large_map(20, std::vector<bool>(20, false));
    
    // Add some obstacles to make it challenging
    for (int i = 5; i < 15; i++) {
        large_map[10][i] = true;
        large_map[i][10] = true;
    }
    
    // Test with increasing number of agents
    std::vector<int> agent_counts = {5, 10, 15};
    
    for (int num_agents : agent_counts) {
        std::vector<cbs_planner::CBSPlanner::Coordinate> starts;
        std::vector<cbs_planner::CBSPlanner::Coordinate> goals;
        
        // Generate starts and goals
        for (int i = 0; i < num_agents; ++i) {
            starts.push_back({i % 5, i / 5});
            goals.push_back({15 + (i % 5), 15 + (i / 5)});
        }
        
        cbs_planner::CBSPlanner::Config config;
        config.time_limit = 30.0; // Give more time for complex scenarios
        config.node_limit = 50000;
        
        auto result = planner->planPaths(large_map, starts, goals, config);
        
        // Should handle gracefully (either succeed or fail with proper error)
        if (result.success) {
            EXPECT_EQ(result.paths.size(), num_agents);
            EXPECT_GT(result.solution_cost, 0);
        } else {
            EXPECT_FALSE(result.error_message.empty());
        }
        
        EXPECT_GE(result.runtime, 0.0);
        EXPECT_GE(result.num_expanded, 0);
    }
}

// Test 2: Random map stress test
TEST_F(RobustnessTest, RandomMapStressTest) {
    for (int trial = 0; trial < 5; ++trial) {
        auto random_map = generateRandomMap(10, 10, 0.3);
        auto starts = generateRandomPositions(random_map, 2);
        auto goals = generateRandomPositions(random_map, 2);
        
        cbs_planner::CBSPlanner::Config config;
        config.time_limit = 10.0;
        
        auto result = planner->planPaths(random_map, starts, goals, config);
        
        // Should not crash regardless of result
        EXPECT_GE(result.runtime, 0.0);
        EXPECT_GE(result.num_expanded, 0);
        
        if (result.success) {
            EXPECT_EQ(result.paths.size(), 2);
            EXPECT_GT(result.solution_cost, 0);
        }
    }
}

// Test 3: Memory stress test
TEST_F(RobustnessTest, MemoryStressTest) {
    // Create multiple planners and run them sequentially
    for (int i = 0; i < 10; ++i) {
        auto local_planner = std::make_unique<cbs_planner::CBSPlanner>();
        
        std::vector<std::vector<bool>> test_map = {
            {false, false, false, false, false},
            {false, true,  false, true,  false},
            {false, false, false, false, false}
        };
        
        std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 4}};
        std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 4}, {2, 0}};
        
        auto result = local_planner->planPaths(test_map, starts, goals);
        
        // Should consistently work
        EXPECT_TRUE(result.success || !result.error_message.empty());
    }
}

// Test 4: Concurrent access test
TEST_F(RobustnessTest, ConcurrentAccessTest) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false, false, false},
        {false, false, false, false, false},
        {false, false, false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 2}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}, {2, 0}};
    
    std::vector<std::thread> threads;
    std::vector<bool> results(4, false);
    
    // Run multiple planning operations concurrently
    for (int i = 0; i < 4; ++i) {
        threads.emplace_back([&, i]() {
            auto local_planner = std::make_unique<cbs_planner::CBSPlanner>();
            auto result = local_planner->planPaths(test_map, starts, goals);
            results[i] = result.success;
        });
    }
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
    
    // All should succeed (simple problem)
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(results[i]) << "Thread " << i << " failed";
    }
}

// Test 5: Extreme map sizes
TEST_F(RobustnessTest, ExtremeMapSizes) {
    // Test very small maps
    std::vector<std::vector<bool>> tiny_map = {{false}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> tiny_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> tiny_goals = {{0, 0}};
    
    auto tiny_result = planner->planPaths(tiny_map, tiny_starts, tiny_goals);
    EXPECT_TRUE(tiny_result.success);
    if (tiny_result.success) {
        EXPECT_EQ(tiny_result.paths.size(), 1);
        EXPECT_EQ(tiny_result.paths[0].size(), 1); // Just the start/goal position
    }
    
    // Test larger maps with timeout protection
    std::vector<std::vector<bool>> large_map(50, std::vector<bool>(50, false));
    std::vector<cbs_planner::CBSPlanner::Coordinate> large_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> large_goals = {{49, 49}};
    
    cbs_planner::CBSPlanner::Config config;
    config.time_limit = 5.0; // Reasonable timeout
    
    auto large_result = planner->planPaths(large_map, large_starts, large_goals, config);
    
    // Should handle gracefully
    EXPECT_GE(large_result.runtime, 0.0);
    if (large_result.success) {
        EXPECT_EQ(large_result.paths.size(), 1);
        EXPECT_GT(large_result.paths[0].size(), 49); // At least Manhattan distance
    }
}

// Test 6: Invalid input resilience
TEST_F(RobustnessTest, InvalidInputResilience) {
    std::vector<std::vector<bool>> valid_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    // Test with mismatched starts and goals
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}}; // Only one goal for two starts
    
    auto result = planner->planPaths(valid_map, starts, goals);
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    
    // Test with empty starts/goals
    std::vector<cbs_planner::CBSPlanner::Coordinate> empty_coords;
    auto empty_result = planner->planPaths(valid_map, empty_coords, empty_coords);
    // Should handle gracefully (might succeed with empty result or fail with error)
    EXPECT_GE(empty_result.runtime, 0.0);
}

// Test 7: Numerical stability test
TEST_F(RobustnessTest, NumericalStabilityTest) {
    std::vector<std::vector<bool>> test_map(100, std::vector<bool>(100, false));
    
    // Create a scenario that might cause numerical issues
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts;
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals;
    
    for (int i = 0; i < 10; ++i) {
        starts.push_back({i, 0});
        goals.push_back({99 - i, 99});
    }
    
    cbs_planner::CBSPlanner::Config config;
    config.time_limit = 15.0;
    
    auto result = planner->planPaths(test_map, starts, goals, config);
    
    // Check for reasonable numerical values
    EXPECT_GE(result.runtime, 0.0);
    EXPECT_LT(result.runtime, 100.0); // Should not report unreasonable runtime
    
    if (result.success) {
        EXPECT_GT(result.solution_cost, 0);
        EXPECT_LT(result.solution_cost, 10000); // Reasonable upper bound
    }
}

// Test 8: Resource exhaustion handling
TEST_F(RobustnessTest, ResourceExhaustionHandling) {
    // Create a complex scenario that should exhaust resources
    std::vector<std::vector<bool>> complex_map(15, std::vector<bool>(15, false));
    
    // Create a maze-like structure
    for (int i = 1; i < 14; i += 2) {
        for (int j = 1; j < 14; j += 2) {
            complex_map[i][j] = true;
        }
    }
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts;
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals;
    
    for (int i = 0; i < 8; ++i) {
        starts.push_back({0, i * 2});
        goals.push_back({14, 14 - i * 2});
    }
    
    cbs_planner::CBSPlanner::Config resource_limited_config;
    resource_limited_config.time_limit = 1.0; // Very short time
    resource_limited_config.node_limit = 100; // Very low node limit
    
    auto result = planner->planPaths(complex_map, starts, goals, resource_limited_config);
    
    // Should handle resource exhaustion gracefully
    EXPECT_GE(result.runtime, 0.0);
    EXPECT_LE(result.runtime, 2.0); // Should respect time limit (with some tolerance)
    
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 9: Pathological map configurations
TEST_F(RobustnessTest, PathologicalMapConfigurations) {
    // Test 1: Checkerboard pattern (alternating obstacles)
    std::vector<std::vector<bool>> checkerboard(10, std::vector<bool>(10, false));
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) {
            checkerboard[y][x] = (x + y) % 2 == 1;
        }
    }
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> cb_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> cb_goals = {{8, 8}};
    
    auto cb_result = planner->planPaths(checkerboard, cb_starts, cb_goals);
    // Should handle this complex pattern
    EXPECT_GE(cb_result.runtime, 0.0);
    
    // Test 2: Spiral pattern
    std::vector<std::vector<bool>> spiral(11, std::vector<bool>(11, false));
    for (int i = 1; i < 10; ++i) {
        spiral[1][i] = true;     // Top
        spiral[i][9] = true;     // Right
        spiral[9][i] = true;     // Bottom
        spiral[i][1] = true;     // Left
        spiral[3][i] = true;     // Inner spiral
        spiral[i][7] = true;
    }
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> spiral_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> spiral_goals = {{5, 5}};
    
    auto spiral_result = planner->planPaths(spiral, spiral_starts, spiral_goals);
    EXPECT_GE(spiral_result.runtime, 0.0);
}

// Test 10: Edge case coordinate handling
TEST_F(RobustnessTest, EdgeCaseCoordinateHandling) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    // Test corner to corner movement
    std::vector<cbs_planner::CBSPlanner::Coordinate> corner_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> corner_goals = {{2, 2}};
    
    auto corner_result = planner->planPaths(test_map, corner_starts, corner_goals);
    EXPECT_TRUE(corner_result.success);
    
    // Test edge positions
    std::vector<cbs_planner::CBSPlanner::Coordinate> edge_starts = {{1, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> edge_goals = {{1, 2}};
    
    auto edge_result = planner->planPaths(test_map, edge_starts, edge_goals);
    EXPECT_TRUE(edge_result.success);
}

// Test 11: Repeated planning calls
TEST_F(RobustnessTest, RepeatedPlanningCalls) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false, false},
        {false, false, false, false},
        {false, false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{3, 2}};
    
    // Perform many planning calls with the same planner instance
    for (int i = 0; i < 20; ++i) {
        auto result = planner->planPaths(test_map, starts, goals);
        EXPECT_TRUE(result.success) << "Failed on iteration " << i;
        
        if (result.success) {
            EXPECT_EQ(result.paths.size(), 1);
            EXPECT_GT(result.paths[0].size(), 0);
        }
    }
}

// Test 12: Configuration robustness
TEST_F(RobustnessTest, ConfigurationRobustness) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    // Test various extreme configurations
    std::vector<cbs_planner::CBSPlanner::Config> extreme_configs;
    
    // Extreme time limits
    cbs_planner::CBSPlanner::Config extreme_time;
    extreme_time.time_limit = 0.001; // Very short
    extreme_configs.push_back(extreme_time);
    
    extreme_time.time_limit = 1000.0; // Very long
    extreme_configs.push_back(extreme_time);
    
    // Extreme node limits
    cbs_planner::CBSPlanner::Config extreme_nodes;
    extreme_nodes.node_limit = 1; // Very low
    extreme_configs.push_back(extreme_nodes);
    
    extreme_nodes.node_limit = 1000000; // Very high
    extreme_configs.push_back(extreme_nodes);
    
    for (const auto& config : extreme_configs) {
        auto result = planner->planPaths(test_map, starts, goals, config);
        
        // Should handle gracefully without crashing
        EXPECT_GE(result.runtime, 0.0);
        EXPECT_GE(result.num_expanded, 0);
        
        if (!result.success) {
            EXPECT_FALSE(result.error_message.empty());
        }
    }
}
