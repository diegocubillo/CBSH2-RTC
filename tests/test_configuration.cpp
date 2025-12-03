#include <gtest/gtest.h>
#include <CBSPlanner.h>
#include <chrono>

class ConfigurationTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
        
        // Standard test map
        test_map = {
            {false, false, false, false, false},
            {false, true,  true,  false, false},
            {false, false, false, false, false},
            {false, false, true,  true,  false},
            {false, false, false, false, false}
        };
        
        // Standard test scenario
        starts = {{0, 0}, {0, 4}};
        goals = {{4, 4}, {4, 0}};
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
    std::vector<std::vector<bool>> test_map;
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts;
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals;
};

// Test 1: Default configuration
TEST_F(ConfigurationTest, DefaultConfiguration) {
    auto config = cbs_planner::CBSPlanner::getDefaultConfig();
    
    EXPECT_EQ(config.time_limit, 60.0);
    EXPECT_EQ(config.node_limit, 100000);
    EXPECT_TRUE(config.prioritize_conflicts);
    EXPECT_TRUE(config.bypass);
    EXPECT_TRUE(config.target_reasoning);
    EXPECT_FALSE(config.mutex_reasoning);
    EXPECT_FALSE(config.disjoint_splitting);
    EXPECT_FALSE(config.rectangle_reasoning);
    EXPECT_FALSE(config.corridor_reasoning);
    EXPECT_FALSE(config.use_sipp);
}

// Test 2: Time limit enforcement
TEST_F(ConfigurationTest, TimeLimitEnforcement) {
    // Create complex scenario that should take some time
    std::vector<std::vector<bool>> complex_map(15, std::vector<bool>(15, false));
    
    // Add obstacles to make it more complex
    for (int i = 2; i < 13; i += 3) {
        for (int j = 2; j < 13; j += 2) {
            complex_map[i][j] = true;
        }
    }
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> complex_starts = {{0, 0}, {0, 14}, {14, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> complex_goals = {{14, 14}, {14, 0}, {0, 14}};
    
    cbs_planner::CBSPlanner::Config config;
    config.time_limit = 0.001; // Very short time limit (1ms)
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto result = planner->planPaths(complex_map, complex_starts, complex_goals, config);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Should respect time limit (allow some tolerance for overhead)
    EXPECT_LT(duration.count(), 100); // Should finish quickly due to timeout
    
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 3: Node limit enforcement
TEST_F(ConfigurationTest, NodeLimitEnforcement) {
    cbs_planner::CBSPlanner::Config config;
    config.node_limit = 5; // Very low node limit
    
    auto result = planner->planPaths(test_map, starts, goals, config);
    
    // With such a low node limit, it should either:
    // 1. Succeed quickly with a simple solution, or
    // 2. Fail due to node limit exceeded
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
    
    // Should not expand more nodes than the limit (with some tolerance)
    EXPECT_LE(result.num_expanded, config.node_limit + 10);
}

// Test 4: Prioritize conflicts enabled vs disabled
TEST_F(ConfigurationTest, PrioritizeConflictsComparison) {
    cbs_planner::CBSPlanner::Config config_with = cbs_planner::CBSPlanner::getDefaultConfig();
    config_with.prioritize_conflicts = true;
    
    cbs_planner::CBSPlanner::Config config_without = config_with;
    config_without.prioritize_conflicts = false;
    
    auto result_with = planner->planPaths(test_map, starts, goals, config_with);
    auto result_without = planner->planPaths(test_map, starts, goals, config_without);
    
    // Both should succeed but might have different performance characteristics
    EXPECT_TRUE(result_with.success);
    EXPECT_TRUE(result_without.success);
    
    // Results should be valid solutions
    if (result_with.success) {
        EXPECT_EQ(result_with.paths.size(), 2);
    }
    if (result_without.success) {
        EXPECT_EQ(result_without.paths.size(), 2);
    }
}

// Test 5: Bypass reasoning enabled vs disabled
TEST_F(ConfigurationTest, BypassReasoningComparison) {
    cbs_planner::CBSPlanner::Config config_with = cbs_planner::CBSPlanner::getDefaultConfig();
    config_with.bypass = true;
    
    cbs_planner::CBSPlanner::Config config_without = config_with;
    config_without.bypass = false;
    
    auto result_with = planner->planPaths(test_map, starts, goals, config_with);
    auto result_without = planner->planPaths(test_map, starts, goals, config_without);
    
    // Both should succeed
    EXPECT_TRUE(result_with.success);
    EXPECT_TRUE(result_without.success);
    
    // Bypass might improve performance
    if (result_with.success && result_without.success) {
        EXPECT_EQ(result_with.paths.size(), 2);
        EXPECT_EQ(result_without.paths.size(), 2);
    }
}

// Test 6: Target reasoning enabled vs disabled
TEST_F(ConfigurationTest, TargetReasoningComparison) {
    cbs_planner::CBSPlanner::Config config_with = cbs_planner::CBSPlanner::getDefaultConfig();
    config_with.target_reasoning = true;
    
    cbs_planner::CBSPlanner::Config config_without = config_with;
    config_without.target_reasoning = false;
    
    auto result_with = planner->planPaths(test_map, starts, goals, config_with);
    auto result_without = planner->planPaths(test_map, starts, goals, config_without);
    
    EXPECT_TRUE(result_with.success);
    EXPECT_TRUE(result_without.success);
}

// Test 7: SIPP vs A* solver comparison
TEST_F(ConfigurationTest, SippVsAStarComparison) {
    cbs_planner::CBSPlanner::Config sipp_config = cbs_planner::CBSPlanner::getDefaultConfig();
    sipp_config.use_sipp = true;
    
    cbs_planner::CBSPlanner::Config astar_config = sipp_config;
    astar_config.use_sipp = false;
    
    auto sipp_result = planner->planPaths(test_map, starts, goals, sipp_config);
    auto astar_result = planner->planPaths(test_map, starts, goals, astar_config);
    
    // Both should succeed and produce valid results
    EXPECT_TRUE(sipp_result.success);
    EXPECT_TRUE(astar_result.success);
    
    if (sipp_result.success && astar_result.success) {
        EXPECT_EQ(sipp_result.paths.size(), 2);
        EXPECT_EQ(astar_result.paths.size(), 2);
        
        // Both should find optimal or near-optimal solutions
        EXPECT_GT(sipp_result.solution_cost, 0);
        EXPECT_GT(astar_result.solution_cost, 0);
    }
}

// Test 8: Rectangle reasoning strategies
TEST_F(ConfigurationTest, RectangleReasoningStrategies) {
    cbs_planner::CBSPlanner::Config config_none = cbs_planner::CBSPlanner::getDefaultConfig();
    config_none.rectangle_reasoning = false;
    
    cbs_planner::CBSPlanner::Config config_enabled = config_none;
    config_enabled.rectangle_reasoning = true;
    
    auto result_none = planner->planPaths(test_map, starts, goals, config_none);
    auto result_enabled = planner->planPaths(test_map, starts, goals, config_enabled);
    
    // Both should succeed
    EXPECT_TRUE(result_none.success);
    EXPECT_TRUE(result_enabled.success);
    
    // Rectangle reasoning might improve performance in some cases
    if (result_enabled.success) {
        EXPECT_EQ(result_enabled.paths.size(), 2);
    }
}

// Test 9: Corridor reasoning strategies
TEST_F(ConfigurationTest, CorridorReasoningStrategies) {
    cbs_planner::CBSPlanner::Config config_none = cbs_planner::CBSPlanner::getDefaultConfig();
    config_none.corridor_reasoning = false;
    
    cbs_planner::CBSPlanner::Config config_enabled = config_none;
    config_enabled.corridor_reasoning = true;
    
    auto result_none = planner->planPaths(test_map, starts, goals, config_none);
    auto result_enabled = planner->planPaths(test_map, starts, goals, config_enabled);
    
    EXPECT_TRUE(result_none.success);
    EXPECT_TRUE(result_enabled.success);
}

// Test 10: Mutex reasoning enabled vs disabled
TEST_F(ConfigurationTest, MutexReasoningComparison) {
    cbs_planner::CBSPlanner::Config config_without = cbs_planner::CBSPlanner::getDefaultConfig();
    config_without.mutex_reasoning = false;
    
    cbs_planner::CBSPlanner::Config config_with = config_without;
    config_with.mutex_reasoning = true;
    
    auto result_without = planner->planPaths(test_map, starts, goals, config_without);
    auto result_with = planner->planPaths(test_map, starts, goals, config_with);
    
    EXPECT_TRUE(result_without.success);
    EXPECT_TRUE(result_with.success);
}

// Test 11: Disjoint splitting enabled vs disabled
TEST_F(ConfigurationTest, DisjointSplittingComparison) {
    cbs_planner::CBSPlanner::Config config_without = cbs_planner::CBSPlanner::getDefaultConfig();
    config_without.disjoint_splitting = false;
    
    cbs_planner::CBSPlanner::Config config_with = config_without;
    config_with.disjoint_splitting = true;
    
    auto result_without = planner->planPaths(test_map, starts, goals, config_without);
    auto result_with = planner->planPaths(test_map, starts, goals, config_with);
    
    EXPECT_TRUE(result_without.success);
    EXPECT_TRUE(result_with.success);
}

// Test 12: Extreme configuration values
TEST_F(ConfigurationTest, ExtremeConfigurationValues) {
    cbs_planner::CBSPlanner::Config extreme_config;
    extreme_config.time_limit = 0.0001;  // Very short
    extreme_config.node_limit = 1;       // Very low
    extreme_config.prioritize_conflicts = false;
    extreme_config.bypass = false;
    extreme_config.target_reasoning = false;
    extreme_config.mutex_reasoning = false;
    extreme_config.disjoint_splitting = false;
    extreme_config.rectangle_reasoning = false;
    extreme_config.corridor_reasoning = false;
    extreme_config.use_sipp = false;
    
    auto result = planner->planPaths(test_map, starts, goals, extreme_config);
    
    // Should handle extreme values gracefully (likely fail but not crash)
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
    
    // Should not crash or produce invalid results
    EXPECT_GE(result.runtime, 0.0);
    EXPECT_GE(result.num_expanded, 0);
}

// Test 13: All reasoning techniques enabled
TEST_F(ConfigurationTest, AllReasoningTechniquesEnabled) {
    cbs_planner::CBSPlanner::Config max_config;
    max_config.time_limit = 30.0;
    max_config.node_limit = 50000;
    max_config.prioritize_conflicts = true;
    max_config.bypass = true;
    max_config.target_reasoning = true;
    max_config.mutex_reasoning = true;
    max_config.disjoint_splitting = true;
    max_config.rectangle_reasoning = true;
    max_config.corridor_reasoning = true;
    max_config.use_sipp = true;
    
    auto result = planner->planPaths(test_map, starts, goals, max_config);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 2);
        EXPECT_GT(result.solution_cost, 0);
    }
}
