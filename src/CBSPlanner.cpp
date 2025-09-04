/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, March 2021
*/

#include "CBSPlanner.h"
#include "CBS.h"
#include "Instance.h"
#include "SpaceTimeAStar.h"
#include "SIPP.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace cbs_planner {

/**
 * @brief Private implementation class (PIMPL pattern)
 */
class CBSPlanner::Impl {
public:
    CBS* cbs_solver = nullptr;
    Instance* instance = nullptr;
    
    ~Impl() {
        cleanup();
    }
    
    void cleanup() {
        if (cbs_solver) {
            cbs_solver->clearSearchEngines();
            delete cbs_solver;
            cbs_solver = nullptr;
        }
        if (instance) {
            delete instance;
            instance = nullptr;
        }
    }
    
    bool createTemporaryAgentFile(const std::vector<Coordinate>& starts,
                                 const std::vector<Coordinate>& goals,
                                 const std::string& filename) {
        if (starts.size() != goals.size()) {
            return false;
        }
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        file << starts.size() << std::endl;
        for (size_t i = 0; i < starts.size(); ++i) {
            file << starts[i].first << "," << starts[i].second << ","
                 << goals[i].first << "," << goals[i].second << ",0" << std::endl;
        }
        
        file.close();
        return true;
    }
    
    bool createTemporaryMapFile(const std::vector<std::vector<bool>>& map_data,
                               const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        int rows = map_data.size();
        int cols = map_data.empty() ? 0 : map_data[0].size();
        
        // Write in simple format: rows,cols followed by grid
        file << rows << "," << cols << std::endl;
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file << (map_data[i][j] ? '@' : '.');
            }
            file << std::endl;
        }
        
        file.close();
        return true;
    }
    
    CBSPlanner::Result convertToResult(bool success) {
        CBSPlanner::Result result;
        result.success = success;
        
        if (!cbs_solver) {
            result.error_message = "CBS solver not initialized";
            return result;
        }
        
        result.runtime = cbs_solver->runtime;
        result.num_expanded = cbs_solver->num_HL_expanded;
        result.num_generated = cbs_solver->num_HL_generated;
        
        if (success && cbs_solver->solution_found) {
            result.solution_cost = cbs_solver->solution_cost;
            
            // Extract paths using savePaths to a temporary file and then read it back
            std::string temp_path_file = "/tmp/cbs_temp_paths.txt";
            cbs_solver->savePaths(temp_path_file);
            
            // Parse the paths file
            std::ifstream file(temp_path_file);
            if (file.is_open()) {
                std::string line;
                while (std::getline(file, line)) {
                    if (line.find("Agent") != std::string::npos) {
                        // Parse line format: "Agent i: (x1,y1)->(x2,y2)->..."
                        size_t colon_pos = line.find(':');
                        if (colon_pos != std::string::npos) {
                            std::string path_str = line.substr(colon_pos + 1);
                            CBSPlanner::Path agent_path;
                            
                            // Find all coordinate pairs
                            size_t pos = 0;
                            while (pos < path_str.length()) {
                                size_t start_paren = path_str.find('(', pos);
                                if (start_paren == std::string::npos) break;
                                
                                size_t end_paren = path_str.find(')', start_paren);
                                if (end_paren == std::string::npos) break;
                                
                                std::string coord_str = path_str.substr(start_paren + 1, end_paren - start_paren - 1);
                                size_t comma_pos = coord_str.find(',');
                                
                                if (comma_pos != std::string::npos) {
                                    try {
                                        int x = std::stoi(coord_str.substr(0, comma_pos));
                                        int y = std::stoi(coord_str.substr(comma_pos + 1));
                                        agent_path.emplace_back(x, y);
                                    } catch (const std::exception&) {
                                        // Skip invalid coordinates
                                    }
                                }
                                
                                pos = end_paren + 1;
                            }
                            
                            result.paths.push_back(std::move(agent_path));
                        }
                    }
                }
                file.close();
            }
            
            // Clean up temporary file
            std::remove(temp_path_file.c_str());
        } else {
            result.error_message = success ? "Solution found but paths not available" : 
                                           "No solution found within limits";
        }
        
        return result;
    }
};

CBSPlanner::CBSPlanner() : pImpl(std::make_unique<Impl>()) {
}

CBSPlanner::~CBSPlanner() = default;

CBSPlanner::Result CBSPlanner::planPaths(const std::string& map_file,
                                        const std::vector<Coordinate>& starts,
                                        const std::vector<Coordinate>& goals,
                                        const Config& config) {
    if (starts.size() != goals.size()) {
        Result result;
        result.success = false;
        result.error_message = "Number of start positions must match number of goal positions";
        return result;
    }
    
    // Handle zero agents case - valid scenario with no planning needed
    if (starts.empty()) {
        Result result;
        result.success = true;
        result.paths = {}; // Empty paths
        result.solution_cost = 0;
        result.runtime = 0.0;
        result.num_expanded = 0;
        result.num_generated = 0;
        return result;
    }
    
    // Clean up previous instances
    pImpl->cleanup();
    
    // Create temporary agent file
    std::string temp_agent_file = "/tmp/cbs_temp_agents.agents";
    if (!pImpl->createTemporaryAgentFile(starts, goals, temp_agent_file)) {
        Result result;
        result.success = false;
        result.error_message = "Failed to create temporary agent file";
        return result;
    }
    
    try {
        // Create instance
        pImpl->instance = new Instance(map_file, temp_agent_file, starts.size(), "", 0, 0, 0, 0);
        
        // Check if instance creation was successful
        if (!pImpl->instance->isValid()) {
            Result result;
            result.success = false;
            result.error_message = "Failed to create instance: " + pImpl->instance->getErrorMessage();
            return result;
        }
        
        // Create CBS solver
        pImpl->cbs_solver = new CBS(*pImpl->instance, config.use_sipp, 0);
        
        // Configure CBS parameters
        pImpl->cbs_solver->setPrioritizeConflicts(config.prioritize_conflicts);
        pImpl->cbs_solver->setBypass(config.bypass);
        pImpl->cbs_solver->setTargetReasoning(config.target_reasoning);
        pImpl->cbs_solver->setMutexReasoning(config.mutex_reasoning);
        pImpl->cbs_solver->setDisjointSplitting(config.disjoint_splitting);
        
        // Handle rectangle reasoning
        if (config.rectangle_reasoning) {
            pImpl->cbs_solver->setRectangleReasoning(rectangle_strategy::R);
        } else {
            pImpl->cbs_solver->setRectangleReasoning(rectangle_strategy::NR);
        }
        
        // Handle corridor reasoning
        if (config.corridor_reasoning) {
            pImpl->cbs_solver->setCorridorReasoning(corridor_strategy::C);
        } else {
            pImpl->cbs_solver->setCorridorReasoning(corridor_strategy::NC);
        }
        
        pImpl->cbs_solver->setNodeLimit(config.node_limit);
        
        // Solve
        bool success = pImpl->cbs_solver->solve(config.time_limit, 0);
        
        // Clean up temporary file
        std::remove(temp_agent_file.c_str());
        
        return pImpl->convertToResult(success);
        
    } catch (const std::exception& e) {
        // Clean up temporary file
        std::remove(temp_agent_file.c_str());
        
        Result result;
        result.success = false;
        result.error_message = std::string("Exception during planning: ") + e.what();
        return result;
    }
}

CBSPlanner::Result CBSPlanner::planPaths(const std::vector<std::vector<bool>>& map_data,
                                        const std::vector<Coordinate>& starts,
                                        const std::vector<Coordinate>& goals,
                                        const Config& config) {
    if (map_data.empty() || map_data[0].empty()) {
        Result result;
        result.success = false;
        result.error_message = "Map data cannot be empty";
        return result;
    }
    
    // Validate coordinates
    int rows = map_data.size();
    int cols = map_data[0].size();
    
    for (const auto& coord : starts) {
        if (!isValidCoordinate(coord, rows, cols)) {
            Result result;
            result.success = false;
            result.error_message = "Invalid start coordinate: (" + 
                                 std::to_string(coord.first) + "," + std::to_string(coord.second) + ")";
            return result;
        }
    }
    
    for (const auto& coord : goals) {
        if (!isValidCoordinate(coord, rows, cols)) {
            Result result;
            result.success = false;
            result.error_message = "Invalid goal coordinate: (" + 
                                 std::to_string(coord.first) + "," + std::to_string(coord.second) + ")";
            return result;
        }
    }
    
    // Create temporary map file
    std::string temp_map_file = "/tmp/cbs_temp_map.map";
    if (!pImpl->createTemporaryMapFile(map_data, temp_map_file)) {
        Result result;
        result.success = false;
        result.error_message = "Failed to create temporary map file";
        return result;
    }
    
    // Use the file-based planning method
    Result result = planPaths(temp_map_file, starts, goals, config);
    
    // Clean up temporary file
    std::remove(temp_map_file.c_str());
    
    return result;
}

CBSPlanner::Config CBSPlanner::getDefaultConfig() {
    return Config{};
}

bool CBSPlanner::isValidCoordinate(const Coordinate& coord, int rows, int cols) {
    return coord.first >= 0 && coord.first < rows && 
           coord.second >= 0 && coord.second < cols;
}

std::string CBSPlanner::pathsToString(const Paths& paths) {
    std::stringstream ss;
    for (size_t i = 0; i < paths.size(); ++i) {
        ss << "Agent " << i << ": ";
        for (size_t j = 0; j < paths[i].size(); ++j) {
            if (j > 0) ss << "->";
            ss << "(" << paths[i][j].first << "," << paths[i][j].second << ")";
        }
        ss << std::endl;
    }
    return ss.str();
}

CBSPlanner::Result CBSPlanner::planPaths(const std::string& map_file,
                                        const std::string& scenario_file,
                                        const Config& config) {
    // Parse scenario file to extract starts and goals
    std::vector<Coordinate> starts, goals;
    if (!parseScenarioFile(scenario_file, starts, goals)) {
        Result result;
        result.success = false;
        result.error_message = "Failed to parse scenario file: " + scenario_file;
        return result;
    }
    
    // Use the existing map_file + starts/goals method
    return planPaths(map_file, starts, goals, config);
}

CBSPlanner::Result CBSPlanner::planPaths(const std::vector<std::vector<bool>>& map_data,
                                        const std::string& scenario_file,
                                        const Config& config) {
    // Parse scenario file to extract starts and goals
    std::vector<Coordinate> starts, goals;
    if (!parseScenarioFile(scenario_file, starts, goals)) {
        Result result;
        result.success = false;
        result.error_message = "Failed to parse scenario file: " + scenario_file;
        return result;
    }
    
    // Use the existing map_data + starts/goals method
    return planPaths(map_data, starts, goals, config);
}

bool CBSPlanner::parseScenarioFile(const std::string& scenario_file,
                                 std::vector<Coordinate>& starts,
                                 std::vector<Coordinate>& goals) {
    std::ifstream file(scenario_file);
    if (!file.is_open()) {
        return false;
    }
    
    starts.clear();
    goals.clear();
    
    std::string line;
    
    // Skip header line if present (version line)
    if (std::getline(file, line)) {
        // Check if this is a version line (starts with "version")
        if (line.find("version") == 0) {
            // This is a header, skip it
        } else {
            // This is data, process it
            file.seekg(0, std::ios::beg); // Go back to beginning
        }
    }
    
    // Parse scenario data
    // Format: bucket map width height startx starty goalx goaly distance
    // We only care about: startx starty goalx goaly
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }
        
        std::istringstream iss(line);
        std::string bucket, map_name;
        int width, height, start_x, start_y, goal_x, goal_y;
        double distance;
        
        // Parse the line: bucket map width height startx starty goalx goaly distance
        if (iss >> bucket >> map_name >> width >> height >> start_x >> start_y >> goal_x >> goal_y >> distance) {
            starts.push_back({start_y, start_x}); // Note: .scen uses (x,y), we use (row,col)
            goals.push_back({goal_y, goal_x});
        } else {
            // Failed to parse this line, continue to next
            continue;
        }
    }
    
    file.close();
    
    // Verify we parsed some agents
    return !starts.empty() && starts.size() == goals.size();
}

} // namespace cbs_planner
