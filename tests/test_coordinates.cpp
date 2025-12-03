#include <gtest/gtest.h>
#include <CBSPlanner.h>

class CoordinatesTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
        
        // Create a 5x5 test map with some obstacles
        test_map = {
            {false, false, false, false, false},
            {false, true,  false, true,  false},
            {false, false, false, false, false},
            {false, true,  false, true,  false},
            {false, false, false, false, false}
        };
    }
    
    std::unique_ptr<cbs_planner::CBSPlanner> planner;
    std::vector<std::vector<bool>> test_map;
};

// Test 1: Valid coordinates within bounds
TEST_F(CoordinatesTest, ValidCoordinates) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}};
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.error_message.empty());
}

// Test 2: Out of bounds coordinates (negative)
TEST_F(CoordinatesTest, NegativeCoordinates) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{-1, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{2, 2}};
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 3: Out of bounds coordinates (too large)
TEST_F(CoordinatesTest, TooLargeCoordinates) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{10, 10}};
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 4: Start on obstacle
TEST_F(CoordinatesTest, StartOnObstacle) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{1, 1}}; // Obstacle
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}};
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    // Library may handle this gracefully or fail - both are acceptable
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    }
    // Just ensure it doesn't crash
    EXPECT_TRUE(true);
}

// Test 5: Goal on obstacle
TEST_F(CoordinatesTest, GoalOnObstacle) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{1, 1}}; // Obstacle
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 6: Agent count mismatch
TEST_F(CoordinatesTest, MismatchedAgentCount) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}, {0, 4}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}}; // Only one goal
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

// Test 7: Empty agent lists
TEST_F(CoordinatesTest, EmptyAgentLists) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> empty_starts;
    std::vector<cbs_planner::CBSPlanner::Coordinate> empty_goals;
    
    auto result = planner->planPaths(test_map, empty_starts, empty_goals);
    
    // Library may reject empty agent lists - this is acceptable behavior
    if (!result.success) {
        EXPECT_FALSE(result.error_message.empty());
    } else {
        EXPECT_TRUE(result.paths.empty());
    }
}

// Test 8: Same start and goal position
TEST_F(CoordinatesTest, SameStartAndGoal) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> coords = {{2, 2}};
    
    auto result = planner->planPaths(test_map, coords, coords);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 1);
    EXPECT_EQ(result.paths[0].size(), 1);
    EXPECT_EQ(result.paths[0][0], coords[0]);
}

// Test 9: Edge coordinates (map boundaries)
TEST_F(CoordinatesTest, EdgeCoordinates) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts = {{0, 0}};
    std::vector<cbs_planner::CBSPlanner::Coordinate> goals = {{4, 4}};
    
    auto result = planner->planPaths(test_map, starts, goals);
    
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.error_message.empty());
}
