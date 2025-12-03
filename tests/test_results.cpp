#include <gtest/gtest.h>
#include <CBSPlanner.h>
#include <set>
#include <algorithm>

class ResultsTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
        
        // Standard test map
        test_map = {
            {false, false, false, false, false},
            {false, true,  false, true,  false},
            {false, false, false, false, false},
            {false, true,  false, true,  false},
            {false, false, false, false, false}
        };
        
        starts = {{0, 0}, {0, 4}};
        goals = {{4, 4}, {4, 0}};
    }
    
    bool isValidPath(const std::vector<cbs_planner::CBSPlanner::Coordinate>& path,
                    const cbs_planner::CBSPlanner::Coordinate& start,
                    const cbs_planner::CBSPlanner::Coordinate& goal) {
        if (path.empty()) return false;
        
        // Check start and goal
        if (path.front().first != start.first || path.front().second != start.second) return false;
        if (path.back().first != goal.first || path.back().second != goal.second) return false;
        
        // Check consecutive moves are valid (adjacent or same position)
        for (size_t i = 1; i < path.size(); ++i) {
            int dx = abs(path[i].first - path[i-1].first);
            int dy = abs(path[i].second - path[i-1].second);
            
            // Valid moves: stay in place or move to adjacent cell
            if (dx + dy > 1) return false;
        }
        
        return true;
    }
    
    bool isValidCoordinate(const cbs_planner::CBSPlanner::Coordinate& coord) {
        return coord.first >= 0 && coord.first < (int)test_map[0].size() &&
               coord.second >= 0 && coord.second < (int)test_map.size() &&
               !test_map[coord.second][coord.first]; // Not an obstacle
    }
    
    bool hasConflict(const std::vector<cbs_planner::CBSPlanner::Coordinate>& path1,
                    const std::vector<cbs_planner::CBSPlanner::Coordinate>& path2) {
        size_t max_time = std::max(path1.size(), path2.size());
        
        for (size_t t = 0; t < max_time; ++t) {
            // Get positions at time t (stay at goal if path is shorter)
            auto pos1 = t < path1.size() ? path1[t] : path1.back();
            auto pos2 = t < path2.size() ? path2[t] : path2.back();
            
            // Vertex conflict
            if (pos1.first == pos2.first && pos1.second == pos2.second) return true;
            
            // Edge conflict (agents swap positions)
            if (t > 0) {
                auto prev1 = t-1 < path1.size() ? path1[t-1] : path1.back();
                auto prev2 = t-1 < path2.size() ? path2[t-1] : path2.back();
                
                if (prev1.first == pos2.first && prev1.second == pos2.second &&
                    prev2.first == pos1.first && prev2.second == pos1.second) {
                    return true;
                }
            }
        }
        
        return false;
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
    std::vector<std::vector<bool>> test_map;
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts;
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals;
};

// Test 1: Basic result structure validation
TEST_F(ResultsTest, BasicResultStructure) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 2);
    EXPECT_GT(result.solution_cost, 0);
    EXPECT_GE(result.runtime, 0.0);
    EXPECT_GT(result.num_expanded, 0);
    EXPECT_TRUE(result.error_message.empty());
}

// Test 2: Path validity verification
TEST_F(ResultsTest, PathValidityVerification) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.paths.size(), 2);
    
    // Check each path individually
    for (size_t i = 0; i < result.paths.size(); ++i) {
        EXPECT_TRUE(isValidPath(result.paths[i], starts[i], goals[i])) 
            << "Path " << i << " is invalid";
        
        // All coordinates should be within bounds and not obstacles
        for (const auto& coord : result.paths[i]) {
            EXPECT_TRUE(isValidCoordinate(coord))
                << "Invalid coordinate (" << coord.first << ", " << coord.second << ") in path " << i;
        }
    }
}

// Test 3: Conflict-free verification
TEST_F(ResultsTest, ConflictFreeVerification) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.paths.size(), 2);
    
    // Check for conflicts between all pairs of paths
    for (size_t i = 0; i < result.paths.size(); ++i) {
        for (size_t j = i + 1; j < result.paths.size(); ++j) {
            EXPECT_FALSE(hasConflict(result.paths[i], result.paths[j]))
                << "Conflict detected between path " << i << " and path " << j;
        }
    }
}

// Test 4: Solution cost calculation
TEST_F(ResultsTest, SolutionCostCalculation) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    ASSERT_TRUE(result.success);
    
    // Calculate expected cost (sum of individual path costs)
    double expected_cost = 0;
    for (const auto& path : result.paths) {
        expected_cost += path.size() - 1; // Cost is number of moves
    }
    
    EXPECT_DOUBLE_EQ(result.solution_cost, expected_cost);
}

// Test 5: Performance metrics validation
TEST_F(ResultsTest, PerformanceMetricsValidation) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    
    // Runtime should be positive and reasonable
    EXPECT_GT(result.runtime, 0.0);
    EXPECT_LT(result.runtime, 10.0); // Should not take more than 10 seconds for simple case
    
    // Expanded nodes should be positive
    EXPECT_GT(result.num_expanded, 0);
    EXPECT_LT(result.num_expanded, 10000); // Reasonable upper bound for simple case
}

// Test 6: Multiple agent result consistency
TEST_F(ResultsTest, MultipleAgentResultConsistency) {
    // Test with different numbers of agents
    std::vector<int> agent_counts = {1, 2, 3, 4};
    
    for (int num_agents : agent_counts) {
        std::vector<cbs_planner::CBSPlanner::Coordinate> test_starts;
        std::vector<cbs_planner::CBSPlanner::Coordinate> test_goals;
        
        for (int i = 0; i < num_agents; ++i) {
            test_starts.push_back({0, i});
            test_goals.push_back({4, 4 - i});
        }
        
        auto result = planner->planPaths(test_map, test_starts, test_goals);
        
        if (result.success) {
            EXPECT_EQ(result.paths.size(), num_agents) 
                << "Incorrect number of paths for " << num_agents << " agents";
            
            // Check all paths are valid
            for (size_t i = 0; i < result.paths.size(); ++i) {
                EXPECT_TRUE(isValidPath(result.paths[i], test_starts[i], test_goals[i]))
                    << "Invalid path for agent " << i << " with " << num_agents << " total agents";
            }
        }
    }
}

// Test 7: Optimal solution verification (simple cases)
TEST_F(ResultsTest, OptimalSolutionVerification) {
    // Simple case: single agent, straight line
    std::vector<std::vector<bool>> simple_map = {
        {false, false, false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> simple_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> simple_goals = {{4, 0}};
    
    auto result = planner->planPaths(simple_map, simple_starts, simple_goals);
    
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.paths.size(), 1);
    
    // Optimal path should be exactly 5 steps (0->1->2->3->4)
    EXPECT_EQ(result.paths[0].size(), 5);
    EXPECT_DOUBLE_EQ(result.solution_cost, 4.0); // 4 moves
}

// Test 8: Consistent results across multiple runs
TEST_F(ResultsTest, ConsistentResultsAcrossRuns) {
    std::vector<cbs_planner::CBSPlanner::Result> results;
    
    // Run the same problem multiple times
    for (int run = 0; run < 5; ++run) {
        auto result = planner->planPaths(test_map, starts, goals);
        results.push_back(result);
    }
    
    // All runs should succeed
    for (const auto& result : results) {
        EXPECT_TRUE(result.success);
    }
    
    // Solution costs should be identical (deterministic optimal solution)
    for (size_t i = 1; i < results.size(); ++i) {
        EXPECT_DOUBLE_EQ(results[0].solution_cost, results[i].solution_cost)
            << "Solution cost differs between run 0 and run " << i;
    }
}

// Test 9: Path length analysis
TEST_F(ResultsTest, PathLengthAnalysis) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    ASSERT_TRUE(result.success);
    
    for (size_t i = 0; i < result.paths.size(); ++i) {
        const auto& path = result.paths[i];
        
        // Path should have at least one position
        EXPECT_GT(path.size(), 0) << "Empty path for agent " << i;
        
        // Calculate Manhattan distance
        int manhattan_dist = abs(goals[i].first - starts[i].first) + abs(goals[i].second - starts[i].second);
        
        // Path length should be at least the Manhattan distance
        EXPECT_GE((int)path.size() - 1, manhattan_dist) 
            << "Path " << i << " is shorter than Manhattan distance";
        
        // Path should be reasonably close to optimal (not more than 3x Manhattan distance)
        EXPECT_LE((int)path.size() - 1, manhattan_dist * 3)
            << "Path " << i << " is unreasonably long";
    }
}

// Test 10: Error message validation for failed cases
TEST_F(ResultsTest, ErrorMessageValidation) {
    // Create impossible scenario
    std::vector<std::vector<bool>> blocked_map = {
        {false, true, false},
        {true,  true, true},
        {false, true, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> blocked_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> blocked_goals = {{2, 2}};
    
    auto result = planner->planPaths(blocked_map, blocked_starts, blocked_goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    EXPECT_EQ(result.paths.size(), 0);
    EXPECT_EQ(result.solution_cost, 0.0);
}

// Test 11: Memory efficiency check
TEST_F(ResultsTest, MemoryEfficiencyCheck) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    ASSERT_TRUE(result.success);
    
    // Paths should not contain duplicate consecutive positions (efficient representation)
    for (size_t i = 0; i < result.paths.size(); ++i) {
        const auto& path = result.paths[i];
        
        // Check for unnecessary duplicate consecutive positions
        int duplicate_count = 0;
        for (size_t j = 1; j < path.size(); ++j) {
            if (path[j].first == path[j-1].first && path[j].second == path[j-1].second) {
                duplicate_count++;
            }
        }
        
        // Some duplicates might be necessary for coordination, but shouldn't be excessive
        EXPECT_LT(duplicate_count, (int)path.size() / 2)
            << "Path " << i << " has too many duplicate consecutive positions";
    }
}

// Test 12: Boundary conditions in results
TEST_F(ResultsTest, BoundaryConditionsInResults) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    ASSERT_TRUE(result.success);
    
    // All coordinates should be within map boundaries
    for (size_t i = 0; i < result.paths.size(); ++i) {
        for (const auto& coord : result.paths[i]) {
            EXPECT_GE(coord.first, 0) << "Negative x coordinate in path " << i;
            EXPECT_GE(coord.second, 0) << "Negative y coordinate in path " << i;
            EXPECT_LT(coord.first, (int)test_map[0].size()) << "X coordinate out of bounds in path " << i;
            EXPECT_LT(coord.second, (int)test_map.size()) << "Y coordinate out of bounds in path " << i;
        }
    }
}

// Test 13: Result completeness check
TEST_F(ResultsTest, ResultCompletenessCheck) {
    auto result = planner->planPaths(test_map, starts, goals);
    
    if (result.success) {
        // All required fields should be properly set
        EXPECT_EQ(result.paths.size(), starts.size());
        EXPECT_GT(result.solution_cost, 0);
        EXPECT_GE(result.runtime, 0.0);
        EXPECT_GT(result.num_expanded, 0);
        EXPECT_TRUE(result.error_message.empty());
        
        // Each path should reach its goal
        for (size_t i = 0; i < result.paths.size(); ++i) {
            EXPECT_FALSE(result.paths[i].empty()) << "Empty path for agent " << i;
            if (!result.paths[i].empty()) {
                const auto& last_pos = result.paths[i].back();
                EXPECT_EQ(last_pos.first, goals[i].first) << "Agent " << i << " didn't reach goal x";
                EXPECT_EQ(last_pos.second, goals[i].second) << "Agent " << i << " didn't reach goal y";
            }
        }
    } else {
        // Failed results should have appropriate fields set
        EXPECT_EQ(result.paths.size(), 0);
        EXPECT_EQ(result.solution_cost, 0.0);
        EXPECT_FALSE(result.error_message.empty());
    }
}
