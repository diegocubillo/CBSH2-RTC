#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include <sys/stat.h>
#include "CBSPlanner.h"

class GenerationTest : public ::testing::Test {
protected:
    void SetUp() override {
        planner = std::make_unique<cbs_planner::CBSPlanner>();
    }

    void TearDown() override {
        // Clean up any test files
        std::vector<std::string> test_files = {
            "test_gen_map.pgm",
            "test_gen_scen.scen",
            "test_gen_scen-1.scen",
            "test_random_instance.pgm",
            "test_random_instance.scen",
            "test_random_instance-1.pgm",
            "test_random_instance-1.scen"
        };
        
        for (const auto& file : test_files) {
            std::remove(file.c_str());
        }
    }

    bool fileExists(const std::string& name) {
        struct stat buffer;   
        return (stat (name.c_str(), &buffer) == 0); 
    }

    std::unique_ptr<cbs_planner::CBSPlanner> planner;
};

// Test 1: Generate Random Scenario on existing map
TEST_F(GenerationTest, GenerateRandomScenario) {
    // Create a dummy map first
    std::ofstream map_file("test_gen_map.pgm");
    map_file << "P2\n3 3\n255\n255 255 255\n255 0 255\n255 255 255\n";
    map_file.close();
    
    bool success = planner->generateRandomScenario("test_gen_map.pgm", 2, "test_gen_scen.scen");
    
    EXPECT_TRUE(success);
    EXPECT_TRUE(fileExists("test_gen_scen.scen"));
    
    // Verify content format (Minimalist)
    std::ifstream scen_file("test_gen_scen.scen");
    std::string line;
    std::getline(scen_file, line);
    EXPECT_EQ(line, "2"); // Num agents
    
    std::getline(scen_file, line);
    // Should be row,col,row,col
    // We can't predict exact values but we can check format
    EXPECT_NE(line.find(','), std::string::npos);
    
    scen_file.close();
}

// Test 2: Generate Random Instance (Map + Scenario)
TEST_F(GenerationTest, GenerateRandomInstance) {
    bool success = planner->generateRandomInstance(5, 5, 2, 3, "test_random_instance.pgm", "test_random_instance.scen");
    
    EXPECT_TRUE(success);
    EXPECT_TRUE(fileExists("test_random_instance.pgm"));
    EXPECT_TRUE(fileExists("test_random_instance.scen"));
    
    // Verify PGM header
    std::ifstream map_file("test_random_instance.pgm");
    std::string line;
    std::getline(map_file, line);
    EXPECT_EQ(line, "P2");
    map_file.close();
    
    // Verify Scenario
    std::ifstream scen_file("test_random_instance.scen");
    std::getline(scen_file, line);
    EXPECT_EQ(line, "3"); // Num agents
    scen_file.close();
}

// Test 3: Unique Filenames
TEST_F(GenerationTest, UniqueFilenames) {
    // Generate first instance
    planner->generateRandomInstance(3, 3, 0, 1, "test_random_instance.pgm", "test_random_instance.scen");
    
    // Generate second instance with same base names
    planner->generateRandomInstance(3, 3, 0, 1, "test_random_instance.pgm", "test_random_instance.scen");
    
    EXPECT_TRUE(fileExists("test_random_instance.pgm"));
    EXPECT_TRUE(fileExists("test_random_instance.scen"));
    EXPECT_TRUE(fileExists("test_random_instance-1.pgm"));
    EXPECT_TRUE(fileExists("test_random_instance-1.scen"));
}

// Test 4: Parse Minimalist Format
TEST_F(GenerationTest, ParseMinimalistFormat) {
    std::ofstream scen_file("test_minimalist.scen");
    scen_file << "2\n";
    scen_file << "0,0,2,2\n";
    scen_file << "0,2,2,0\n";
    scen_file.close();
    
    std::vector<cbs_planner::CBSPlanner::Coordinate> starts, goals;
    bool success = cbs_planner::CBSPlanner::parseScenarioFile("test_minimalist.scen", starts, goals);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(starts.size(), 2);
    EXPECT_EQ(goals.size(), 2);
    
    EXPECT_EQ(starts[0], std::make_pair(0, 0));
    EXPECT_EQ(goals[0], std::make_pair(2, 2));
    
    std::remove("test_minimalist.scen");
}
