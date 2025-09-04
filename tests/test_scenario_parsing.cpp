#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>  // For std::remove
#include "CBSPlanner.h"

class ScenarioParsingTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
    }

    void TearDown() override {
        // Clean up any test files
        std::vector<std::string> test_files = {
            "test_scenario.scen",
            "test_scenario_with_header.scen", 
            "test_empty.scen",
            "test_malformed.scen",
            "simple_test.map"
        };
        
        for (const auto& file : test_files) {
            std::remove(file.c_str());
        }
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
};

// Test 1: Basic scenario file parsing
TEST_F(ScenarioParsingTest, BasicScenarioParsing) {
    // Create a simple scenario file
    std::ofstream scen_file("test_scenario.scen");
    scen_file << "0 simple_test.map 5 5 0 0 4 4 5.6568542494924\n";
    scen_file << "1 simple_test.map 5 5 1 0 3 4 4.2426406871193\n";
    scen_file << "2 simple_test.map 5 5 0 1 4 3 4.2426406871193\n";
    scen_file.close();

    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    bool success = cbs_planner::CBSPlanner::parseScenarioFile("test_scenario.scen", starts, goals);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(starts.size(), 3);
    EXPECT_EQ(goals.size(), 3);
    
    // Verify coordinates (remember .scen uses x,y but we use row,col)
    EXPECT_EQ(starts[0], std::make_pair(0, 0)); // (x=0,y=0) -> (row=0,col=0)
    EXPECT_EQ(goals[0], std::make_pair(4, 4));   // (x=4,y=4) -> (row=4,col=4)
    
    EXPECT_EQ(starts[1], std::make_pair(0, 1)); // (x=1,y=0) -> (row=0,col=1)  
    EXPECT_EQ(goals[1], std::make_pair(4, 3));   // (x=3,y=4) -> (row=4,col=3)
    
    EXPECT_EQ(starts[2], std::make_pair(1, 0)); // (x=0,y=1) -> (row=1,col=0)
    EXPECT_EQ(goals[2], std::make_pair(3, 4));   // (x=4,y=3) -> (row=3,col=4)
}

// Test 2: Scenario file with version header
TEST_F(ScenarioParsingTest, ScenarioWithHeader) {
    std::ofstream scen_file("test_scenario_with_header.scen");
    scen_file << "version 1\n";
    scen_file << "0 simple_test.map 5 5 0 0 4 4 5.6568542494924\n";
    scen_file << "1 simple_test.map 5 5 1 0 3 4 4.2426406871193\n";
    scen_file.close();

    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    bool success = cbs_planner::CBSPlanner::parseScenarioFile("test_scenario_with_header.scen", starts, goals);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(starts.size(), 2);
    EXPECT_EQ(goals.size(), 2);
    
    EXPECT_EQ(starts[0], std::make_pair(0, 0));
    EXPECT_EQ(goals[0], std::make_pair(4, 4));
    
    EXPECT_EQ(starts[1], std::make_pair(0, 1));
    EXPECT_EQ(goals[1], std::make_pair(4, 3));
}

// Test 3: Empty scenario file
TEST_F(ScenarioParsingTest, EmptyScenarioFile) {
    std::ofstream scen_file("test_empty.scen");
    scen_file.close();

    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    bool success = cbs_planner::CBSPlanner::parseScenarioFile("test_empty.scen", starts, goals);
    
    EXPECT_FALSE(success);
    EXPECT_EQ(starts.size(), 0);
    EXPECT_EQ(goals.size(), 0);
}

// Test 4: Non-existent scenario file
TEST_F(ScenarioParsingTest, NonExistentFile) {
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    bool success = cbs_planner::CBSPlanner::parseScenarioFile("nonexistent.scen", starts, goals);
    
    EXPECT_FALSE(success);
}

// Test 5: Malformed scenario file
TEST_F(ScenarioParsingTest, MalformedScenarioFile) {
    std::ofstream scen_file("test_malformed.scen");
    scen_file << "version 1\n";
    scen_file << "invalid line format\n";
    scen_file << "0 simple_test.map 5 5 0 0\n"; // Missing goal coordinates
    scen_file << "1 simple_test.map 5 5 1 0 3 4 4.2426406871193\n"; // Valid line
    scen_file.close();

    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    bool success = cbs_planner::CBSPlanner::parseScenarioFile("test_malformed.scen", starts, goals);
    
    EXPECT_TRUE(success); // Should succeed with the one valid line
    EXPECT_EQ(starts.size(), 1);
    EXPECT_EQ(goals.size(), 1);
    
    EXPECT_EQ(starts[0], std::make_pair(0, 1));
    EXPECT_EQ(goals[0], std::make_pair(4, 3));
}

// Test 6: Complete integration test with planning
TEST_F(ScenarioParsingTest, IntegrationWithPlanning) {
    // Create a simple 3x3 map
    std::ofstream map_file("simple_test.map");
    map_file << "3,3\n";
    map_file << "...\n";
    map_file << "...\n";
    map_file << "...\n";
    map_file.close();
    
    // Create scenario file
    std::ofstream scen_file("test_scenario.scen");
    scen_file << "0 simple_test.map 3 3 0 0 2 2 2.8284271247462\n";
    scen_file << "1 simple_test.map 3 3 0 2 2 0 2.8284271247462\n";
    scen_file.close();

    auto result = planner->planPaths("simple_test.map", "test_scenario.scen");
    
    EXPECT_TRUE(result.success);
    if (result.success) {
        EXPECT_EQ(result.paths.size(), 2);
        
        // Verify paths are not empty
        EXPECT_GT(result.paths[0].size(), 0);
        EXPECT_GT(result.paths[1].size(), 0);
        
        // Verify start and end positions
        EXPECT_EQ(result.paths[0][0], std::make_pair(0, 0)); // Agent 0 start
        EXPECT_EQ(result.paths[0].back(), std::make_pair(2, 2)); // Agent 0 goal
        
        EXPECT_EQ(result.paths[1][0], std::make_pair(2, 0)); // Agent 1 start  
        EXPECT_EQ(result.paths[1].back(), std::make_pair(0, 2)); // Agent 1 goal
    } else {
        std::cout << "Planning failed: " << result.error_message << std::endl;
    }
}

// Test 7: Scenario parsing with comments and empty lines
TEST_F(ScenarioParsingTest, ScenarioWithCommentsAndEmptyLines) {
    std::ofstream scen_file("test_scenario.scen");
    scen_file << "version 1\n";
    scen_file << "# This is a comment\n";
    scen_file << "\n"; // Empty line
    scen_file << "0 simple_test.map 5 5 0 0 4 4 5.6568542494924\n";
    scen_file << "   \n"; // Whitespace line
    scen_file << "# Another comment\n";
    scen_file << "1 simple_test.map 5 5 1 0 3 4 4.2426406871193\n";
    scen_file.close();

    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    bool success = cbs_planner::CBSPlanner::parseScenarioFile("test_scenario.scen", starts, goals);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(starts.size(), 2);
    EXPECT_EQ(goals.size(), 2);
}

// Test 8: Error handling for invalid scenario file with planning
TEST_F(ScenarioParsingTest, PlanningWithInvalidScenario) {
    auto result = planner->planPaths("nonexistent.map", "nonexistent.scen");
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    EXPECT_TRUE(result.error_message.find("Failed to parse scenario file") != std::string::npos);
}

// Test 9: Test planPaths with map_data and scenario_file
TEST_F(ScenarioParsingTest, PlanningWithMapDataAndScenarioFile) {
    // Create a simple scenario file
    std::ofstream scen_file("test_scenario.scen");
    scen_file << "0 test.map 5 5 0 0 4 4 5.6568542494924\n";
    scen_file << "1 test.map 5 5 4 0 0 4 5.6568542494924\n";
    scen_file.close();

    // Create a simple 5x5 map data
    std::vector<std::vector<bool>> map_data = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    auto result = planner->planPaths(map_data, "test_scenario.scen");
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 2);
    EXPECT_GT(result.solution_cost, 0);
    
    // Verify that paths start and end at expected coordinates
    EXPECT_FALSE(result.paths[0].empty());
    EXPECT_FALSE(result.paths[1].empty());
    
    EXPECT_EQ(result.paths[0].front(), std::make_pair(0, 0)); // Start position
    EXPECT_EQ(result.paths[0].back(), std::make_pair(4, 4));  // Goal position
    
    EXPECT_EQ(result.paths[1].front(), std::make_pair(0, 4)); // Start position
    EXPECT_EQ(result.paths[1].back(), std::make_pair(4, 0));  // Goal position
}

// Test 10: Test planPaths with map_data and invalid scenario_file
TEST_F(ScenarioParsingTest, PlanningWithMapDataAndInvalidScenario) {
    // Create a simple 5x5 map data
    std::vector<std::vector<bool>> map_data = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    auto result = planner->planPaths(map_data, "nonexistent.scen");
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
    EXPECT_TRUE(result.error_message.find("Failed to parse scenario file") != std::string::npos);
}

// Test 11: Test planPaths with map_file and scenario_file (existing function verification)
TEST_F(ScenarioParsingTest, PlanningWithMapFileAndScenarioFile) {
    // Create a simple scenario file
    std::ofstream scen_file("test_scenario.scen");
    scen_file << "0 simple_test.map 5 5 0 0 4 4 5.6568542494924\n";
    scen_file << "1 simple_test.map 5 5 4 0 0 4 5.6568542494924\n";
    scen_file.close();

    // Create a simple map file
    std::ofstream map_file("simple_test.map");
    map_file << "type octile\n";
    map_file << "height 5\n";
    map_file << "width 5\n";
    map_file << "map\n";
    map_file << ".....\n";
    map_file << ".@@..\n";
    map_file << ".....\n";
    map_file << "..@@.\n";
    map_file << ".....\n";
    map_file.close();

    auto result = planner->planPaths("simple_test.map", "test_scenario.scen");
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.paths.size(), 2);
    EXPECT_GT(result.solution_cost, 0);
}
