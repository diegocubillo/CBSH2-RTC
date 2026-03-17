/*
 * CBSTA.h - CBS with Task Assignment (CBS-TA)
 * 
 * Based on: Hoenig et al., "Conflict-Based Search with Optimal Task Assignment"
 * AAMAS 2018
 * 
 * This class extends CBS to handle Task Assignment and Path Finding (TAPF)
 * by transforming the search tree into a search forest, where each root
 * represents a different task assignment.
 */

#pragma once

#include "CBS.h"
#include "TaskAssignment.h"
#include <memory>
#include <optional>

/**
 * @brief CBS with Task Assignment (CBS-TA)
 * 
 * Extends CBS from MAPF to TAPF by:
 * 1. Transforming single search tree into search forest
 * 2. Each root node represents a different task assignment
 * 3. Root nodes are expanded on-demand with next-best assignment
 * 
 * All CBSH2-RTC optimizations are preserved:
 * - Rectangle reasoning
 * - Corridor reasoning
 * - Target reasoning
 * - Mutex reasoning
 * - Bypass
 * - Heuristics (CG, DG, WDG)
 */
class CBSTA {
public:
    /**
     * @brief Construct a CBS-TA solver
     * 
     * @param instance The instance containing map and agent starts
     * @param goal_locations All potential goal locations (M goals, M >= N agents)
     * @param allowed_matrix allowed[i][j] = true if agent i can reach goal j
     * @param sipp Use SIPP for low-level search (vs Space-Time A*)
     * @param screen Verbosity level (0=silent, 1=stats, 2=verbose)
     */
    CBSTA(const Instance& instance, 
          const std::vector<int>& goal_locations,
          const std::vector<std::vector<bool>>& allowed_matrix,
          bool sipp = false, 
          int screen = 0);
    
    ~CBSTA();
    
    // Prevent copying
    CBSTA(const CBSTA&) = delete;
    CBSTA& operator=(const CBSTA&) = delete;
    
    /////////////////////////////////////////////////////////////////////////////////////////
    // Configuration (same as CBS)
    void setHeuristicType(heuristics_type h);
    void setPrioritizeConflicts(bool p);
    void setRectangleReasoning(rectangle_strategy r);
    void setCorridorReasoning(corridor_strategy c);
    void setTargetReasoning(bool t);
    void setMutexReasoning(bool m);
    void setDisjointSplitting(bool d);
    void setBypass(bool b);
    void setNodeLimit(int n);
    void setSavingStats(bool s);
    
    /////////////////////////////////////////////////////////////////////////////////////////
    // Solve
    
    /**
     * @brief Solve the TAPF problem
     * 
     * Finds optimal task assignment and collision-free paths.
     * 
     * @param time_limit Maximum runtime in seconds
     * @param cost_lowerbound Lower bound on solution cost
     * @param cost_upperbound Upper bound on solution cost
     * @return true if solution found, false otherwise
     */
    bool solve(double time_limit, int cost_lowerbound = 0, 
               int cost_upperbound = MAX_COST);
    
    /////////////////////////////////////////////////////////////////////////////////////////
    // Results
    
    /**
     * @brief Get the optimal task assignment
     * @return Vector where assignment[agent] = goal_id
     */
    std::vector<int> getOptimalAssignment() const;
    
    /**
     * @brief Check if solution was found
     */
    bool isSolutionFound() const { return solution_found_; }
    
    /**
     * @brief Get solution cost (sum of individual costs)
     */
    int getSolutionCost() const { return solution_cost_; }
    
    /**
     * @brief Get runtime in seconds
     */
    double getRuntime() const { return runtime_; }
    
    /**
     * @brief Get number of high-level nodes expanded
     */
    uint64_t getNumHLExpanded() const { return num_HL_expanded_; }
    
    /**
     * @brief Get number of high-level nodes generated
     */
    uint64_t getNumHLGenerated() const { return num_HL_generated_; }
    
    /**
     * @brief Get number of task assignments evaluated (root nodes expanded)
     */
    uint64_t getNumAssignmentsTried() const { return num_assignments_tried_; }
    
    /**
     * @brief Get the computed paths
     * @return Vector of paths, one per agent
     */
    std::vector<Path> getPaths() const;
    
    /**
     * @brief Save results to CSV file
     */
    void saveResults(const std::string& fileName, const std::string& instanceName) const;
    
    /**
     * @brief Print paths to stdout
     */
    void printPaths() const;

private:
    // Problem definition
    const Instance& instance_;
    std::vector<int> all_goals_;  // M potential goal locations
    std::vector<std::vector<bool>> allowed_matrix_;  // N x M
    int num_agents_;
    int num_goals_;
    bool use_sipp_;
    int screen_;
    
    // Task assignment
    std::unique_ptr<cbs_planner::TaskAssignment> task_assignment_;
    std::vector<int> optimal_assignment_;
    
    // Cost matrix: cost_matrix_[agent][goal] = shortest path distance
    std::vector<std::vector<int>> cost_matrix_;
    
    // CBS solver (one instance, reused with different assignments)
    std::unique_ptr<CBS> cbs_solver_;
    
    // Solution state
    bool solution_found_ = false;
    int solution_cost_ = -2;
    double runtime_ = 0.0;
    uint64_t num_HL_expanded_ = 0;
    uint64_t num_HL_generated_ = 0;
    uint64_t num_assignments_tried_ = 0;
    std::vector<Path> solution_paths_;
    
    // Search forest management
    struct RootNode {
        std::vector<int> assignment;
        int lower_bound;  // Assignment cost (ignoring conflicts)
        bool expanded = false;
    };
    
    // Priority queue for root nodes (min-heap by lower bound)
    std::vector<RootNode> root_nodes_;
    
    // Internal methods
    
    /**
     * @brief Compute cost matrix using single-agent shortest paths
     */
    void computeCostMatrix();
    
    /**
     * @brief Create a CBS instance for a specific assignment
     * @param assignment The task assignment to use
     * @return New CBS solver configured for this assignment
     */
    std::unique_ptr<CBS> createCBSForAssignment(const std::vector<int>& assignment);
    
    /**
     * @brief Get name of solver for logging
     */
    std::string getSolverName() const;
};
