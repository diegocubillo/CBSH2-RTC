/*
 * TaskAssignment.h - K-best assignment enumeration for CBS-TA
 * 
 * Based on: Hoenig et al., "Conflict-Based Search with Optimal Task Assignment"
 * AAMAS 2018
 * 
 * This module implements the Hungarian method for optimal assignment and
 * K-best enumeration for generating assignments on demand.
 */

#pragma once

#include <vector>
#include <set>
#include <boost/optional.hpp>
#include <queue>
#include <functional>
#include <limits>

namespace cbs_planner {

// Assignment: assignment[agent_id] = goal_id
using Assignment = std::vector<int>;

// Infinity cost for forbidden assignments
constexpr int ASSIGNMENT_INF = std::numeric_limits<int>::max() / 2;

/**
 * @brief Node in the assignment enumeration tree
 * 
 * Used by the K-best assignment algorithm to partition the solution space.
 * Each node represents a subset of assignments with certain agent-goal pairs
 * included (forced) or excluded (forbidden).
 */
struct AssignmentNode {
    std::set<std::pair<int, int>> included;  // Must include (agent, goal)
    std::set<std::pair<int, int>> excluded;  // Cannot include (agent, goal)
    Assignment solution;                      // The optimal assignment for this partition
    int cost;                                 // Cost of the solution
    
    // Comparison for priority queue (min-heap by cost)
    bool operator>(const AssignmentNode& other) const {
        return cost > other.cost;
    }
};

/**
 * @brief Task Assignment module for CBS-TA
 * 
 * Computes optimal task assignments using the Hungarian method and
 * enumerates K-best assignments on demand for the CBS-TA search forest.
 */
class TaskAssignment {
public:
    /**
     * @brief Construct a TaskAssignment solver
     * 
     * @param cost_matrix Cost matrix C[agent][goal] where C[i][j] is the cost
     *                    for agent i to reach goal j (typically shortest path distance)
     * @param allowed_matrix A[agent][goal] where A[i][j] is true if agent i can
     *                       be assigned to goal j
     */
    TaskAssignment(const std::vector<std::vector<int>>& cost_matrix,
                   const std::vector<std::vector<bool>>& allowed_matrix);
    
    /**
     * @brief Get the first (optimal) assignment
     * 
     * Initializes the enumeration and returns the globally optimal assignment.
     * 
     * @return The optimal assignment minimizing total cost
     */
    Assignment getFirstAssignment();
    
    /**
     * @brief Get the next best assignment
     * 
     * Returns the next best assignment after the previously returned one.
     * Uses lazy enumeration - only expands nodes as needed.
     * 
     * @return The next best assignment, or nullopt if no more exist
     */
    boost::optional<Assignment> getNextAssignment();
    
    /**
     * @brief Compute the cost of an assignment
     * 
     * @param assignment The assignment to evaluate
     * @return Total cost (sum of individual agent costs)
     */
    int getAssignmentCost(const Assignment& assignment) const;
    
    /**
     * @brief Check if an assignment is valid
     * 
     * @param assignment The assignment to check
     * @return True if all agents are assigned to allowed goals
     */
    bool isValidAssignment(const Assignment& assignment) const;
    
    /**
     * @brief Get the number of agents
     */
    int getNumAgents() const { return num_agents_; }
    
    /**
     * @brief Get the number of goals
     */
    int getNumGoals() const { return num_goals_; }

private:
    int num_agents_;
    int num_goals_;
    std::vector<std::vector<int>> cost_matrix_;
    std::vector<std::vector<bool>> allowed_matrix_;
    
    // Priority queue for assignment enumeration (ASG_OPEN in paper)
    std::priority_queue<AssignmentNode, 
                       std::vector<AssignmentNode>,
                       std::greater<AssignmentNode>> assignment_open_;
    
    bool initialized_ = false;
    
    /**
     * @brief Compute optimal assignment with constraints
     * 
     * Finds the optimal assignment given:
     * - included: set of (agent, goal) pairs that MUST be in the solution
     * - excluded: set of (agent, goal) pairs that CANNOT be in the solution
     * 
     * @param included Required assignments
     * @param excluded Forbidden assignments
     * @return Optimal assignment respecting constraints, or empty if infeasible
     */
    Assignment constrainedAssignment(
        const std::set<std::pair<int, int>>& included,
        const std::set<std::pair<int, int>>& excluded);
    
    /**
     * @brief Hungarian method for optimal assignment
     * 
     * Solves the assignment problem using O(n^3) Hungarian algorithm.
     * 
     * @param cost Modified cost matrix with constraints applied
     * @return Optimal assignment, or empty if no feasible assignment exists
     */
    Assignment hungarianMethod(const std::vector<std::vector<int>>& cost);
    
    /**
     * @brief Apply constraints to cost matrix
     * 
     * @param included Set cost to 0 for these pairs
     * @param excluded Set cost to infinity for these pairs
     * @return Modified cost matrix
     */
    std::vector<std::vector<int>> applyConstraints(
        const std::set<std::pair<int, int>>& included,
        const std::set<std::pair<int, int>>& excluded) const;
};

} // namespace cbs_planner
