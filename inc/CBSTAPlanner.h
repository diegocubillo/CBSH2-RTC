/*
 * CBSTAPlanner.h - High-level API for CBS with Task Assignment
 * 
 * Based on: Hoenig et al., "Conflict-Based Search with Optimal Task Assignment"
 * AAMAS 2018
 * 
 * This class provides a simplified interface for solving Task Assignment and
 * Path Finding (TAPF) problems, similar to CBSPlanner for MAPF.
 */

#pragma once

#include <vector>
#include <string>
#include <memory>

namespace cbs_planner {

/**
 * @brief Main interface for CBS-TA based task assignment and path planning
 * 
 * This class provides a simplified interface to the CBS-TA algorithm for
 * solving TAPF problems. It handles the complexity of the underlying
 * CBS-TA implementation and provides a clean API.
 * 
 * Thread-safety: This class is NOT thread-safe. Create separate instances
 * for concurrent use.
 */
class CBSTAPlanner {
public:
    // Type aliases for clarity
    using Coordinate = std::pair<int, int>;  // (row, col)
    using Path = std::vector<Coordinate>;
    using Paths = std::vector<Path>;
    using Assignment = std::vector<int>;  // assignment[agent] = goal_index
    
    /**
     * @brief Configuration options for the planner
     * 
     * Inherits all CBS options plus TAPF-specific settings.
     */
    struct Config {
        // Time and node limits
        double time_limit;              // Time limit in seconds
        int node_limit;                 // Maximum high-level nodes to expand
        
        // CBS optimizations
        bool prioritize_conflicts;      // Use conflict prioritization
        bool target_reasoning;          // Use target reasoning
        bool disjoint_splitting;        // Use disjoint splitting
        bool rectangle_reasoning;       // Use rectangle reasoning
        bool corridor_reasoning;        // Use corridor reasoning
        bool mutex_reasoning;           // Use mutex reasoning
        bool bypass;                    // Use bypass optimization
        
        // Low-level search
        bool use_sipp;                  // Use SIPP (vs Space-Time A*)
        
        // Verbosity (0=silent, 1=stats, 2=verbose)
        int screen;
        
        // Default constructor with default values
        Config() : time_limit(60.0), node_limit(100000),
                   prioritize_conflicts(true), target_reasoning(true),
                   disjoint_splitting(false), rectangle_reasoning(false),
                   corridor_reasoning(false), mutex_reasoning(false),
                   bypass(true), use_sipp(false), screen(0) {}
    };
    
    /**
     * @brief Result of a planning request
     */
    struct Result {
        bool success = false;           // Whether a solution was found
        Paths paths;                    // Collision-free paths for each agent
        Assignment assignment;          // Optimal task assignment
        int solution_cost = -1;         // Total solution cost (sum of path lengths)
        double runtime = 0.0;           // Runtime in seconds
        int num_expanded = 0;           // Number of high-level nodes expanded
        int num_generated = 0;          // Number of high-level nodes generated
        int num_assignments_tried = 0;  // Number of task assignments evaluated
        std::string error_message;      // Error message if planning failed
    };
    
    /**
     * @brief Constructor
     */
    CBSTAPlanner();
    
    /**
     * @brief Destructor
     */
    ~CBSTAPlanner();
    
    // Disable copy
    CBSTAPlanner(const CBSTAPlanner&) = delete;
    CBSTAPlanner& operator=(const CBSTAPlanner&) = delete;
    
    /**
     * @brief Plan paths with optimal task assignment
     * 
     * Solves the TAPF problem: find optimal assignment of agents to goals
     * and collision-free paths for all agents.
     * 
     * @param map_file Path to map file (.map format)
     * @param starts Start positions for N agents
     * @param goals Potential goal positions (M goals, M >= N)
     * @param allowed N x M matrix where allowed[i][j] = true if agent i can go to goal j
     * @param config Planning configuration
     * @return Result containing paths, assignment, and statistics
     */
    Result planPathsWithAssignment(
        const std::string& map_file,
        const std::vector<Coordinate>& starts,
        const std::vector<Coordinate>& goals,
        const std::vector<std::vector<bool>>& allowed,
        const Config& config = Config());
    
    /**
     * @brief Plan paths with optimal task assignment (in-memory map)
     * 
     * @param map_data 2D grid where true = obstacle, false = free
     * @param starts Start positions for N agents
     * @param goals Potential goal positions (M goals, M >= N)
     * @param allowed N x M matrix where allowed[i][j] = true if agent i can go to goal j
     * @param config Planning configuration
     * @return Result containing paths, assignment, and statistics
     */
    Result planPathsWithAssignment(
        const std::vector<std::vector<bool>>& map_data,
        const std::vector<Coordinate>& starts,
        const std::vector<Coordinate>& goals,
        const std::vector<std::vector<bool>>& allowed,
        const Config& config = Config());
    
    /**
     * @brief Plan paths with all agents allowed to reach all goals
     * 
     * Convenience method when there are no assignment restrictions.
     * 
     * @param map_file Path to map file
     * @param starts Start positions for N agents
     * @param goals Potential goal positions (M goals, M >= N)
     * @param config Planning configuration
     * @return Result containing paths, assignment, and statistics
     */
    Result planPathsWithAssignment(
        const std::string& map_file,
        const std::vector<Coordinate>& starts,
        const std::vector<Coordinate>& goals,
        const Config& config = Config());
    
    /**
     * @brief Plan paths with all agents allowed to reach all goals (in-memory map)
     */
    Result planPathsWithAssignment(
        const std::vector<std::vector<bool>>& map_data,
        const std::vector<Coordinate>& starts,
        const std::vector<Coordinate>& goals,
        const Config& config = Config());
    
    /**
     * @brief Get default configuration
     */
    static Config getDefaultConfig();
    
    /**
     * @brief Check if coordinate is valid for given map dimensions
     */
    static bool isValidCoordinate(const Coordinate& coord, int rows, int cols);
    
    /**
     * @brief Convert paths to string representation
     */
    static std::string pathsToString(const Paths& paths);
    
    /**
     * @brief Convert assignment to string representation
     */
    static std::string assignmentToString(const Assignment& assignment, 
                                          const std::vector<Coordinate>& goals);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace cbs_planner
