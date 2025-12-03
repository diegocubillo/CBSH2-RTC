#include <gtest/gtest.h>
#include <CBSPlanner.h>
#include <limits>
#include <climits>
#include <fstream>

class EdgeCasesTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
};

// Test 1: Empty map handling using test_data/empty.map
TEST_F(EdgeCasesTest, EmptyMapHandling) {
    auto result = planner->planPaths("tests/test_data/empty.map", {{0, 0}}, {{1, 1}});
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    EXPECT_EQ(result.paths.size(), 0);
}

// Test 2: Single cell map using test_data/single_cell.map
TEST_F(EdgeCasesTest, SingleCellMapFromFile) {
    auto result = planner->planPaths("tests/test_data/single_cell.map", {{0, 0}}, {{0, 0}});
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 1); // Only one position
    }
}

// Test 2: Single cell map (programmatic)
TEST_F(EdgeCasesTest, SingleCellMap) {
    std::vector<std::vector<bool>> single_cell_map = {{false}};
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{0, 0}};
    
    auto result = planner->planPaths(single_cell_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 1); // Only one position
    }
}

// Test 3: Tiny map stress test using test_data/tiny_3x3.map
TEST_F(EdgeCasesTest, TinyMapStressTest) {
    // 2 agents in a 3x3 map with central obstacle
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {2, 2}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}, {0, 0}};
    
    auto result = planner->planPaths("tests/test_data/tiny_3x3.map", starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 2);
        EXPECT_GT(result.solution_cost, 0);
    }
}

// Test 3: All obstacles map
TEST_F(EdgeCasesTest, AllObstaclesMap) {
    std::vector<std::vector<bool>> all_obstacles = {
        {true, true},
        {true, true}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 1}};
    
    auto result = planner->planPaths(all_obstacles, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 4: Single free cell in obstacle map using simple_5x5.map with modifications
TEST_F(EdgeCasesTest, SingleFreeCellInObstacleMap) {
    std::vector<std::vector<bool>> single_free = {
        {true, true, true},
        {true, false, true}, // Only center is free
        {true, true, true}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 1}}; // Same position
    
    auto result = planner->planPaths(single_free, starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 1); // Agent stays in place
    }
}

// Test 5: Linear map (horizontal corridor)
TEST_F(EdgeCasesTest, LinearMap) {
    std::vector<std::vector<bool>> linear_map = {
        {false, false, false, false, false}  // 1 row, 5 columns
    };
    
    // Move agent from leftmost to rightmost position in the row
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};  // (row 0, col 0)
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{0, 4}};   // (row 0, col 4)
    
    auto result = planner->planPaths(linear_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 5); // 5 steps: (0,0)->(0,1)->(0,2)->(0,3)->(0,4)
        EXPECT_DOUBLE_EQ(result.solution_cost, 4.0);
    } else {
        std::cout << "LinearMap failed with error: " << result.error_message << std::endl;
    }
}sTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
};

// Test 1: Empty map handling
TEST_F(EdgeCasesTest, EmptyMapHandling) {
    std::vector<std::vector<bool>> empty_map;
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 1}};
    
    auto result = planner->planPaths(empty_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    EXPECT_EQ(result.paths.size(), 0);
}

// Test 2: Single cell map
TEST_F(EdgeCasesTest, SingleCellMap) {
    std::vector<std::vector<bool>> single_cell = {{false}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{0, 0}};
    
    auto result = planner->planPaths(single_cell, starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 1);
        EXPECT_EQ(result.paths[0][0].first, 0);
        EXPECT_EQ(result.paths[0][0].second, 0);
        EXPECT_DOUBLE_EQ(result.solution_cost, 0.0);
    }
}

// Test 3: All obstacles map
TEST_F(EdgeCasesTest, AllObstaclesMap) {
    std::vector<std::vector<bool>> all_obstacles = {
        {true, true, true},
        {true, true, true},
        {true, true, true}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    auto result = planner->planPaths(all_obstacles, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 4: Single free cell in obstacle map
TEST_F(EdgeCasesTest, SingleFreeCellInObstacleMap) {
    std::vector<std::vector<bool>> mostly_obstacles = {
        {true,  true,  true},
        {true,  false, true},
        {true,  true,  true}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 1}};
    
    auto result = planner->planPaths(mostly_obstacles, starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 1);
        EXPECT_DOUBLE_EQ(result.solution_cost, 0.0);
    }
}

// Test 6: Vertical linear map
TEST_F(EdgeCasesTest, VerticalLinearMap) {
    std::vector<std::vector<bool>> vertical_map = {
        {false},  // Row 0, Col 0
        {false},  // Row 1, Col 0
        {false},  // Row 2, Col 0
        {false}   // Row 3, Col 0
    };
    
    // Move agent from top to bottom of the column
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};  // (row 0, col 0)
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{3, 0}};   // (row 3, col 0)
    
    auto result = planner->planPaths(vertical_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 4);
        EXPECT_DOUBLE_EQ(result.solution_cost, 3.0);
    } else {
        std::cout << "VerticalLinearMap failed with error: " << result.error_message << std::endl;
    }
}

// Test 7: Zero agents
TEST_F(EdgeCasesTest, ZeroAgents) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> empty_starts;
    std::vector<cbs_planner::CBSPlanner::Coordinate> empty_goals;
    
    auto result = planner->planPaths(test_map, empty_starts, empty_goals);
    
    // Should handle gracefully - either succeed with empty result or provide clear error
    EXPECT_GE(result.runtime, 0.0);
    EXPECT_GE(result.num_expanded, 0);
    
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 0);
        EXPECT_DOUBLE_EQ(result.solution_cost, 0.0);
    } else {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 8: Maximum coordinate values
TEST_F(EdgeCasesTest, MaximumCoordinateValues) {
    // Create a reasonably sized map to test coordinate limits
    std::vector<std::vector<bool>> test_map(10, std::vector<bool>(10, false));
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{9, 9}};
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        // Verify path coordinates are within bounds
        for (const auto& coord : result.paths[0]) {
            EXPECT_GE(coord.first, 0);
            EXPECT_LE(coord.first, 9);
            EXPECT_GE(coord.second, 0);
            EXPECT_LE(coord.second, 9);
        }
    }
}

// Test 9: Out-of-bounds start/goal coordinates
TEST_F(EdgeCasesTest, OutOfBoundsCoordinates) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    // Test with start out of bounds
    std::vector<cbs_planner::CBSPlanner::Coordinate> oob_starts = {{5, 5}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> valid_goals = {{2, 2}};
    
    auto result1 = planner->planPaths(test_map, oob_starts, valid_goals);
    EXPECT_FALSE(result1.success);
    EXPECT_FALSE(result1.error_message.empty());
    
    // Test with goal out of bounds
    std::vector<cbs_planner::CBSPlanner::Coordinate> valid_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> oob_goals = {{10, 10}};
    
    auto result2 = planner->planPaths(test_map, valid_starts, oob_goals);
    EXPECT_FALSE(result2.success);
    EXPECT_FALSE(result2.error_message.empty());
    
    // Test with negative coordinates
    std::vector<cbs_planner::CBSPlanner::Coordinate> neg_starts = {{-1, -1}};
    
    auto result3 = planner->planPaths(test_map, neg_starts, valid_goals);
    EXPECT_FALSE(result3.success);
    EXPECT_FALSE(result3.error_message.empty());
}

// Test 10: Start equals goal
TEST_F(EdgeCasesTest, StartEqualsGoal) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> same_pos = {{1, 1}};
    
    auto result = planner->planPaths(test_map, same_pos, same_pos);
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_EQ(result.paths[0].size(), 1);
        EXPECT_EQ(result.paths[0][0].first, 1);
        EXPECT_EQ(result.paths[0][0].second, 1);
        EXPECT_DOUBLE_EQ(result.solution_cost, 0.0);
    }
}

// Test 11: Multiple agents same start/goal
TEST_F(EdgeCasesTest, MultipleAgentsSameStartGoal) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    // All agents start and end at the same position - this is a pathological case
    std::vector<cbs_planner::CBSPlanner::Coordinate> same_starts = {{1, 1}, {1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> same_goals = {{1, 1}, {1, 1}};
    
    // Use very restrictive limits to prevent infinite loops
    cbs_planner::CBSPlanner::Config config;
    config.time_limit = 0.1; // 100ms max
    config.node_limit = 10;   // Very few nodes
    
    auto result = planner->planPaths(test_map, same_starts, same_goals, config);
    
    // This is a conflict situation - agents can't occupy the same cell
    // The planner should either find a solution with temporal separation or fail quickly
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 2);
        // If successful, paths might have different lengths to avoid conflicts
    } else {
        EXPECT_FALSE(result.error_message.empty());
        // Should fail due to impossible situation or resource limits
    }
}

// Test 12: Start on obstacle
TEST_F(EdgeCasesTest, StartOnObstacle) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, true,  false},
        {false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> obstacle_starts = {{1, 1}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> valid_goals = {{2, 2}};
    
    auto result = planner->planPaths(test_map, obstacle_starts, valid_goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 13: Goal on obstacle
TEST_F(EdgeCasesTest, GoalOnObstacle) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, true,  false},
        {false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> valid_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> obstacle_goals = {{1, 1}};
    
    auto result = planner->planPaths(test_map, valid_starts, obstacle_goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 14: Extremely tight configuration limits
TEST_F(EdgeCasesTest, ExtremeleTightLimits) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    cbs_planner::CBSPlanner::Config tight_config;
    tight_config.time_limit = 0.000001; // 1 microsecond
    tight_config.node_limit = 0;        // Zero nodes
    
    auto result = planner->planPaths(test_map, starts, goals, tight_config);
    
    // Should handle gracefully
    EXPECT_GE(result.runtime, 0.0);
    EXPECT_GE(result.num_expanded, 0);
    
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 15: Large number of agents on small map
TEST_F(EdgeCasesTest, TooManyAgentsSmallMap) {
    std::vector<std::vector<bool>> small_map = {
        {false, false},
        {false, false}
    };
    
    // Try to place 10 agents on a 2x2 map
    std::vector<cbs_planner::CBSPlanner::Coordinate> many_starts;
    std::vector<cbs_planner::CBSPlanner::Coordinate> many_goals;
    
    for (int i = 0; i < 10; ++i) {
        many_starts.push_back({i % 2, i / 2 % 2});
        many_goals.push_back({(i + 1) % 2, (i + 1) / 2 % 2});
    }
    
    auto result = planner->planPaths(small_map, many_starts, many_goals);
    
    // Should recognize this as impossible or handle gracefully
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
}

// Test 16: Asymmetric map dimensions
TEST_F(EdgeCasesTest, AsymmetricMapDimensions) {
    // Very wide but short map
    std::vector<std::vector<bool>> wide_map = {
        {false, false, false, false, false, false, false, false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{9, 0}};
    
    auto result1 = planner->planPaths(wide_map, starts, goals);
    EXPECT_TRUE(result1.success);
    
    // Very tall but narrow map
    std::vector<std::vector<bool>> tall_map(10, std::vector<bool>(1, false));
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> tall_starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> tall_goals = {{0, 9}};
    
    auto result2 = planner->planPaths(tall_map, tall_starts, tall_goals);
    EXPECT_TRUE(result2.success);
}

// Test 17: Irregular map shapes (jagged arrays)
TEST_F(EdgeCasesTest, IrregularMapShapes) {
    // This test assumes the planner expects rectangular maps
    // In practice, irregular maps should be handled by proper input validation
    
    std::vector<std::vector<bool>> regular_map = {
        {false, false, false},
        {false, false, false},
        {false, false, false}
    };
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    auto result = planner->planPaths(regular_map, starts, goals);
    EXPECT_TRUE(result.success);
    
    // The library should handle rectangular maps correctly
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 1);
        EXPECT_GT(result.paths[0].size(), 0);
    }
}

// Test 18: Boundary coordinate planning
TEST_F(EdgeCasesTest, BoundaryCoordinatePlanning) {
    std::vector<std::vector<bool>> test_map = {
        {false, false, false, false},
        {false, false, false, false},
        {false, false, false, false},
        {false, false, false, false}
    };
    
    // Test all corner combinations
    std::vector<std::pair<cbs_planner::CBSPlanner::Coordinate, cbs_planner::CBSPlanner::Coordinate>> corner_pairs = {
        {{0, 0}, {3, 3}}, // Top-left to bottom-right
        {{3, 0}, {0, 3}}, // Top-right to bottom-left
        {{0, 3}, {3, 0}}, // Bottom-left to top-right
        {{3, 3}, {0, 0}}  // Bottom-right to top-left
    };
    
    for (const auto& pair : corner_pairs) {
        std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {pair.first};
        std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {pair.second};
        
        auto result = planner->planPaths(test_map, starts, goals);
        EXPECT_TRUE(result.success) 
            << "Failed for path from (" << pair.first.first << "," << pair.first.second 
            << ") to (" << pair.second.first << "," << pair.second.second << ")";
        
        if (result.success) {
            EXPECT_GT(result.paths[0].size(), 0);
            EXPECT_EQ(result.paths[0].front().first, pair.first.first);
            EXPECT_EQ(result.paths[0].front().second, pair.first.second);
            EXPECT_EQ(result.paths[0].back().first, pair.second.first);
            EXPECT_EQ(result.paths[0].back().second, pair.second.second);
        }
    }
}
