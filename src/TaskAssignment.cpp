/*
 * TaskAssignment.cpp - K-best assignment enumeration for CBS-TA
 * 
 * Based on: Hoenig et al., "Conflict-Based Search with Optimal Task Assignment"
 * AAMAS 2018
 * 
 * Implementation of Hungarian method and K-best enumeration algorithms.
 */

#include "TaskAssignment.h"
#include <algorithm>
#include <numeric>
#include <cassert>
#include <iostream>

namespace cbs_planner {

TaskAssignment::TaskAssignment(
    const std::vector<std::vector<int>>& cost_matrix,
    const std::vector<std::vector<bool>>& allowed_matrix)
    : cost_matrix_(cost_matrix), allowed_matrix_(allowed_matrix) {
    
    num_agents_ = static_cast<int>(cost_matrix.size());
    num_goals_ = num_agents_ > 0 ? static_cast<int>(cost_matrix[0].size()) : 0;
    
    // Validate dimensions
    assert(num_agents_ > 0 && "Cost matrix must have at least one agent");
    assert(num_goals_ >= num_agents_ && "Must have at least as many goals as agents");
    assert(allowed_matrix.size() == static_cast<size_t>(num_agents_));
    
    for (int i = 0; i < num_agents_; i++) {
        assert(cost_matrix[i].size() == static_cast<size_t>(num_goals_));
        assert(allowed_matrix[i].size() == static_cast<size_t>(num_goals_));
    }
}

Assignment TaskAssignment::getFirstAssignment() {
    // Clear any previous state
    while (!assignment_open_.empty()) {
        assignment_open_.pop();
    }
    
    // Create first assignment node with no constraints
    AssignmentNode root;
    root.solution = constrainedAssignment(root.included, root.excluded);
    root.cost = getAssignmentCost(root.solution);
    
    if (!root.solution.empty()) {
        assignment_open_.push(root);
    }
    
    initialized_ = true;
    return root.solution;
}

boost::optional<Assignment> TaskAssignment::getNextAssignment() {
    if (!initialized_) {
        return {};
    }
    
    if (assignment_open_.empty()) {
        return {};
    }
    
    // Pop the current best node (Algorithm 3 from paper)
    AssignmentNode P = assignment_open_.top();
    assignment_open_.pop();
    
    // Expand P to generate child nodes (partition the solution space)
    // For each agent i not in P.included, create a new partition
    for (int i = 0; i < num_agents_; i++) {
        // Check if agent i is already fixed (in included set)
        bool agent_fixed = false;
        for (const auto& ag_pair : P.included) {
            if (ag_pair.first == i) {
                agent_fixed = true;
                break;
            }
        }
        
        if (!agent_fixed) {
            // Create child node Q
            AssignmentNode Q;
            
            // Q.O = P.O ∪ {P.solution[i]} - exclude current assignment for agent i
            Q.excluded = P.excluded;
            Q.excluded.insert({i, P.solution[i]});
            
            // Q.I = P.I ∪ {P.solution[j] : j < i} - include all previous agents' assignments
            Q.included = P.included;
            for (int j = 0; j < i; j++) {
                Q.included.insert({j, P.solution[j]});
            }
            
            // Find optimal assignment with new constraints
            Q.solution = constrainedAssignment(Q.included, Q.excluded);
            
            if (!Q.solution.empty()) {
                Q.cost = getAssignmentCost(Q.solution);
                assignment_open_.push(Q);
            }
        }
    }
    
    // Return the next best assignment from the queue
    if (!assignment_open_.empty()) {
        return assignment_open_.top().solution;
    }
    
    return {};
}

int TaskAssignment::getAssignmentCost(const Assignment& assignment) const {
    if (assignment.empty()) {
        return ASSIGNMENT_INF;
    }
    
    int total_cost = 0;
    for (int agent = 0; agent < num_agents_; agent++) {
        if (agent >= static_cast<int>(assignment.size())) {
            return ASSIGNMENT_INF;
        }
        int goal = assignment[agent];
        if (goal < 0 || goal >= num_goals_) {
            return ASSIGNMENT_INF;
        }
        total_cost += cost_matrix_[agent][goal];
    }
    return total_cost;
}

bool TaskAssignment::isValidAssignment(const Assignment& assignment) const {
    if (static_cast<int>(assignment.size()) != num_agents_) {
        return false;
    }
    
    std::vector<bool> goal_used(num_goals_, false);
    
    for (int agent = 0; agent < num_agents_; agent++) {
        int goal = assignment[agent];
        if (goal < 0 || goal >= num_goals_) {
            return false;
        }
        if (!allowed_matrix_[agent][goal]) {
            return false;
        }
        if (goal_used[goal]) {
            return false;  // Multiple agents assigned to same goal
        }
        goal_used[goal] = true;
    }
    
    return true;
}

std::vector<std::vector<int>> TaskAssignment::applyConstraints(
    const std::set<std::pair<int, int>>& included,
    const std::set<std::pair<int, int>>& excluded) const {
    
    std::vector<std::vector<int>> cost = cost_matrix_;
    
    // Apply allowed matrix constraints
    for (int i = 0; i < num_agents_; i++) {
        for (int j = 0; j < num_goals_; j++) {
            if (!allowed_matrix_[i][j]) {
                cost[i][j] = ASSIGNMENT_INF;
            }
        }
    }
    
    // Apply excluded constraints (set cost to infinity)
    for (const auto& ag_pair : excluded) {
        cost[ag_pair.first][ag_pair.second] = ASSIGNMENT_INF;
    }
    
    // Note: included constraints are handled by the Hungarian method
    // by pre-assigning those agents and removing them from the problem
    
    return cost;
}

Assignment TaskAssignment::constrainedAssignment(
    const std::set<std::pair<int, int>>& included,
    const std::set<std::pair<int, int>>& excluded) {
    
    // Apply constraints to cost matrix
    auto cost = applyConstraints(included, excluded);
    
    // For included assignments, set their row to infinity except the assigned goal
    // This forces the Hungarian method to pick that goal for that agent
    for (const auto& ag_pair : included) {
        for (int j = 0; j < num_goals_; j++) {
            if (j != ag_pair.second) {
                cost[ag_pair.first][j] = ASSIGNMENT_INF;
            } else {
                cost[ag_pair.first][j] = cost_matrix_[ag_pair.first][ag_pair.second];  // Use original cost
            }
        }
    }
    
    return hungarianMethod(cost);
}

Assignment TaskAssignment::hungarianMethod(const std::vector<std::vector<int>>& cost) {
    /*
     * Hungarian Algorithm (Kuhn-Munkres) for minimum cost assignment
     * 
     * This implementation handles rectangular matrices (more goals than agents)
     * by padding with dummy rows if necessary.
     * 
     * Time complexity: O(n^3) where n = max(num_agents, num_goals)
     */
    
    const int n = num_agents_;
    const int m = num_goals_;
    
    // We solve a square problem of size max(n, m)
    const int sz = std::max(n, m);
    
    // Create padded cost matrix (pad with zeros for dummy agents)
    std::vector<std::vector<int>> c(sz, std::vector<int>(sz, 0));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            c[i][j] = cost[i][j];
        }
    }
    
    // u[i] = potential for row i, v[j] = potential for column j
    std::vector<int> u(sz + 1, 0), v(sz + 1, 0);
    
    // p[j] = row assigned to column j, way[j] = previous column in augmenting path
    std::vector<int> p(sz + 1, 0), way(sz + 1, 0);
    
    for (int i = 1; i <= sz; i++) {
        // minv[j] = minimum slack for column j
        std::vector<int> minv(sz + 1, ASSIGNMENT_INF);
        std::vector<bool> used(sz + 1, false);
        
        p[0] = i;
        int j0 = 0;  // Virtual column
        
        do {
            used[j0] = true;
            int i0 = p[j0];
            int delta = ASSIGNMENT_INF;
            int j1 = 0;
            
            for (int j = 1; j <= sz; j++) {
                if (!used[j]) {
                    // Cost from adjusted cost matrix (1-indexed internally)
                    int cur = c[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            
            // Check for infeasible assignment
            if (delta >= ASSIGNMENT_INF) {
                return {};  // No feasible assignment
            }
            
            // Update potentials
            for (int j = 0; j <= sz; j++) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            
            j0 = j1;
        } while (p[j0] != 0);
        
        // Augment along the path
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }
    
    // Extract assignment (only for real agents)
    Assignment result(n, -1);
    for (int j = 1; j <= sz; j++) {
        if (p[j] > 0 && p[j] <= n) {
            int agent = p[j] - 1;
            int goal = j - 1;
            
            // Only assign if within valid range and not infinite cost
            if (goal < m && cost[agent][goal] < ASSIGNMENT_INF) {
                result[agent] = goal;
            }
        }
    }
    
    // Verify all agents are assigned
    for (int i = 0; i < n; i++) {
        if (result[i] < 0) {
            return {};  // Some agent couldn't be assigned
        }
    }
    
    return result;
}

} // namespace cbs_planner
