/*
 * CBSTAPlanner.cpp - High-level API for CBS with Task Assignment
 * 
 * Based on: Hoenig et al., "Conflict-Based Search with Optimal Task Assignment"
 * AAMAS 2018
 */

#include "CBSTAPlanner.h"
#include "CBSTA.h"
#include "Instance.h"
#include "SpaceTimeAStar.h"
#include "SIPP.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <sys/stat.h>

namespace cbs_planner {

// Defined in CBSPlanner.cpp; shared so the Heuristic -> heuristics_type mapping
// is not duplicated.
heuristics_type toHeuristicsType(CBSPlanner::Heuristic h);

// Helper to check if file exists
static bool fileExists(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

// Helper to get unique filename
static std::string getUniqueFilename(const std::string& base_name) {
    std::string filename = base_name;
    int counter = 0;
    while (fileExists(filename)) {
        std::ostringstream oss;
        oss << base_name << "." << counter++;
        filename = oss.str();
    }
    return filename;
}

/**
 * @brief Private implementation class (PIMPL pattern)
 */
class CBSTAPlanner::Impl {
public:
    CBSTA* cbsta_solver = nullptr;
    Instance* instance = nullptr;
    
    ~Impl() {
        cleanup();
    }
    
    void cleanup() {
        if (cbsta_solver) {
            delete cbsta_solver;
            cbsta_solver = nullptr;
        }
        if (instance) {
            delete instance;
            instance = nullptr;
        }
    }
    
    bool createTemporaryAgentFile(const std::vector<CBSTAPlanner::Coordinate>& starts,
                                  const std::vector<CBSTAPlanner::Coordinate>& goals,
                                  const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        file << "version 1" << std::endl;
        for (size_t i = 0; i < starts.size(); i++) {
            // Use first goal as placeholder - CBSTA will override with assignment
            int goal_idx = i < goals.size() ? i : 0;
            file << "0\t" << "temp.map" << "\t0\t0\t"
                 << starts[i].second << "\t" << starts[i].first << "\t"
                 << goals[goal_idx].second << "\t" << goals[goal_idx].first << "\t0" << std::endl;
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
        
        int rows = static_cast<int>(map_data.size());
        int cols = rows > 0 ? static_cast<int>(map_data[0].size()) : 0;
        
        file << "type octile" << std::endl;
        file << "height " << rows << std::endl;
        file << "width " << cols << std::endl;
        file << "map" << std::endl;
        
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                file << (map_data[r][c] ? '@' : '.');
            }
            file << std::endl;
        }
        file.close();
        return true;
    }
    
    CBSTAPlanner::Result convertToResult(CBSTA* solver, 
                                         const std::vector<CBSTAPlanner::Coordinate>& goals,
                                         int num_agents) {
        CBSTAPlanner::Result result;
        
        if (solver && solver->isSolutionFound()) {
            result.success = true;
            result.solution_cost = solver->getSolutionCost();
            result.runtime = solver->getRuntime();
            result.num_expanded = static_cast<int>(solver->getNumHLExpanded());
            result.num_generated = static_cast<int>(solver->getNumHLGenerated());
            result.assignment = solver->getOptimalAssignment();
            result.num_assignments_tried = static_cast<int>(solver->getNumAssignmentsTried());
            
            // Get paths from solver
            auto paths = solver->getPaths();
            result.paths.resize(paths.size());
            for (size_t i = 0; i < paths.size(); i++) {
                result.paths[i].reserve(paths[i].size());
                for (const auto& entry : paths[i]) {
                    // Convert location to coordinate
                    if (instance) {
                        int row = instance->getRowCoordinate(entry.location);
                        int col = instance->getColCoordinate(entry.location);
                        result.paths[i].emplace_back(row, col);
                    }
                }
            }
        } else {
            result.success = false;
            result.error_message = "CBS-TA failed to find a solution";
            if (solver) {
                result.runtime = solver->getRuntime();
                result.num_expanded = static_cast<int>(solver->getNumHLExpanded());
                result.num_generated = static_cast<int>(solver->getNumHLGenerated());
                result.num_assignments_tried = static_cast<int>(solver->getNumAssignmentsTried());
            }
        }
        
        return result;
    }
};

CBSTAPlanner::CBSTAPlanner() : pimpl_(std::make_unique<Impl>()) {}

CBSTAPlanner::~CBSTAPlanner() = default;

CBSTAPlanner::Result CBSTAPlanner::planPathsWithAssignment(
    const std::string& map_file,
    const std::vector<Coordinate>& starts,
    const std::vector<Coordinate>& goals,
    const std::vector<std::vector<bool>>& allowed,
    const Config& config) {
    
    Result result;
    pimpl_->cleanup();
    
    // Validate inputs
    int num_agents = static_cast<int>(starts.size());
    int num_goals = static_cast<int>(goals.size());
    
    if (num_agents == 0) {
        result.error_message = "No agents specified";
        return result;
    }
    if (num_goals < num_agents) {
        result.error_message = "Number of goals must be >= number of agents";
        return result;
    }
    if (static_cast<int>(allowed.size()) != num_agents) {
        result.error_message = "Allowed matrix rows must equal number of agents";
        return result;
    }
    for (int i = 0; i < num_agents; i++) {
        if (static_cast<int>(allowed[i].size()) != num_goals) {
            result.error_message = "Allowed matrix columns must equal number of goals";
            return result;
        }
    }
    
    // Create temporary agent file
    std::string temp_agent_file = getUniqueFilename("/tmp/cbsta_agents.scen");
    if (!pimpl_->createTemporaryAgentFile(starts, goals, temp_agent_file)) {
        result.error_message = "Failed to create temporary agent file";
        return result;
    }
    
    try {
        // Create instance
        pimpl_->instance = new Instance(map_file, temp_agent_file, num_agents);
        
        if (!pimpl_->instance->isValid()) {
            result.error_message = pimpl_->instance->getErrorMessage();
            std::remove(temp_agent_file.c_str());
            return result;
        }
        
        // Convert goal coordinates to locations
        std::vector<int> goal_locations(num_goals);
        for (int j = 0; j < num_goals; j++) {
            goal_locations[j] = pimpl_->instance->linearizeCoordinate(goals[j].first, goals[j].second);
        }
        
        // Create CBS-TA solver
        pimpl_->cbsta_solver = new CBSTA(*pimpl_->instance, goal_locations, allowed,
                                         config.use_sipp, config.screen);
        
        // Configure solver
        pimpl_->cbsta_solver->setHeuristicType(toHeuristicsType(config.heuristic));
        pimpl_->cbsta_solver->setNodeLimit(config.node_limit);
        pimpl_->cbsta_solver->setPrioritizeConflicts(config.prioritize_conflicts);
        pimpl_->cbsta_solver->setTargetReasoning(config.target_reasoning);
        pimpl_->cbsta_solver->setDisjointSplitting(config.disjoint_splitting);
        pimpl_->cbsta_solver->setRectangleReasoning(
            config.rectangle_reasoning ? rectangle_strategy::RM : rectangle_strategy::NR);
        pimpl_->cbsta_solver->setCorridorReasoning(
            config.corridor_reasoning ? corridor_strategy::C : corridor_strategy::NC);
        pimpl_->cbsta_solver->setMutexReasoning(config.mutex_reasoning);
        pimpl_->cbsta_solver->setBypass(config.bypass);
        
        // Solve
        pimpl_->cbsta_solver->solve(config.time_limit);
        
        // Convert result
        result = pimpl_->convertToResult(pimpl_->cbsta_solver, goals, num_agents);
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    }
    
    // Cleanup temporary file
    std::remove(temp_agent_file.c_str());
    
    return result;
}

CBSTAPlanner::Result CBSTAPlanner::planPathsWithAssignment(
    const std::vector<std::vector<bool>>& map_data,
    const std::vector<Coordinate>& starts,
    const std::vector<Coordinate>& goals,
    const std::vector<std::vector<bool>>& allowed,
    const Config& config) {
    
    // Create temporary map file
    std::string temp_map_file = getUniqueFilename("/tmp/cbsta_map.map");
    if (!pimpl_->createTemporaryMapFile(map_data, temp_map_file)) {
        Result result;
        result.error_message = "Failed to create temporary map file";
        return result;
    }
    
    // Call the file-based version
    auto result = planPathsWithAssignment(temp_map_file, starts, goals, allowed, config);
    
    // Cleanup
    std::remove(temp_map_file.c_str());
    
    return result;
}

CBSTAPlanner::Result CBSTAPlanner::planPathsWithAssignment(
    const std::string& map_file,
    const std::vector<Coordinate>& starts,
    const std::vector<Coordinate>& goals,
    const Config& config) {
    
    // Create allowed matrix with all true
    int num_agents = static_cast<int>(starts.size());
    int num_goals = static_cast<int>(goals.size());
    std::vector<std::vector<bool>> allowed(num_agents, std::vector<bool>(num_goals, true));
    
    return planPathsWithAssignment(map_file, starts, goals, allowed, config);
}

CBSTAPlanner::Result CBSTAPlanner::planPathsWithAssignment(
    const std::vector<std::vector<bool>>& map_data,
    const std::vector<Coordinate>& starts,
    const std::vector<Coordinate>& goals,
    const Config& config) {
    
    // Create allowed matrix with all true
    int num_agents = static_cast<int>(starts.size());
    int num_goals = static_cast<int>(goals.size());
    std::vector<std::vector<bool>> allowed(num_agents, std::vector<bool>(num_goals, true));
    
    return planPathsWithAssignment(map_data, starts, goals, allowed, config);
}

CBSTAPlanner::Config CBSTAPlanner::getDefaultConfig() {
    return Config();
}

bool CBSTAPlanner::isValidCoordinate(const Coordinate& coord, int rows, int cols) {
    return coord.first >= 0 && coord.first < rows &&
           coord.second >= 0 && coord.second < cols;
}

std::string CBSTAPlanner::pathsToString(const Paths& paths) {
    std::ostringstream oss;
    for (size_t i = 0; i < paths.size(); i++) {
        oss << "Agent " << i << ": ";
        for (size_t t = 0; t < paths[i].size(); t++) {
            oss << "(" << paths[i][t].first << "," << paths[i][t].second << ")";
            if (t < paths[i].size() - 1) oss << " -> ";
        }
        oss << "\n";
    }
    return oss.str();
}

std::string CBSTAPlanner::assignmentToString(const Assignment& assignment,
                                             const std::vector<Coordinate>& goals) {
    std::ostringstream oss;
    for (size_t i = 0; i < assignment.size(); i++) {
        int goal_idx = assignment[i];
        oss << "Agent " << i << " -> Goal " << goal_idx;
        if (goal_idx >= 0 && static_cast<size_t>(goal_idx) < goals.size()) {
            oss << " at (" << goals[goal_idx].first << "," << goals[goal_idx].second << ")";
        }
        oss << "\n";
    }
    return oss.str();
}

} // namespace cbs_planner
