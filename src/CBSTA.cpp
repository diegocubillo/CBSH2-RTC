/*
 * CBSTA.cpp - CBS with Task Assignment (CBS-TA) Implementation
 * 
 * Based on: Hoenig et al., "Conflict-Based Search with Optimal Task Assignment"
 * AAMAS 2018
 */

#include "CBSTA.h"
#include "SpaceTimeAStar.h"
#include "SIPP.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>

CBSTA::CBSTA(const Instance& instance,
             const std::vector<int>& goal_locations,
             const std::vector<std::vector<bool>>& allowed_matrix,
             bool sipp,
             int screen)
    : instance_(instance),
      all_goals_(goal_locations),
      allowed_matrix_(allowed_matrix),
      use_sipp_(sipp),
      screen_(screen) {
    
    num_agents_ = instance.getDefaultNumberOfAgents();
    num_goals_ = static_cast<int>(goal_locations.size());
    
    // Validate dimensions
    if (num_goals_ < num_agents_) {
        throw std::invalid_argument("Number of goals must be >= number of agents");
    }
    if (static_cast<int>(allowed_matrix.size()) != num_agents_) {
        throw std::invalid_argument("Allowed matrix rows must equal number of agents");
    }
    for (int i = 0; i < num_agents_; i++) {
        if (static_cast<int>(allowed_matrix[i].size()) != num_goals_) {
            throw std::invalid_argument("Allowed matrix columns must equal number of goals");
        }
    }
    
    // Compute cost matrix using shortest paths
    computeCostMatrix();
    
    // Initialize task assignment solver
    task_assignment_ = std::make_unique<cbs_planner::TaskAssignment>(
        cost_matrix_, allowed_matrix_);
}

CBSTA::~CBSTA() {
    // Clean up search engines owned by the last CBS solver
    if (cbs_solver_) {
        cbs_solver_->clearSearchEngines();
    }
}

void CBSTA::setHeuristicType(heuristics_type h) {
    cfg_heuristic_type_ = h;
}

void CBSTA::setPrioritizeConflicts(bool p) {
    cfg_PC_ = p;
}

void CBSTA::setRectangleReasoning(rectangle_strategy r) {
    cfg_rectangle_ = r;
}

void CBSTA::setCorridorReasoning(corridor_strategy c) {
    cfg_corridor_ = c;
}

void CBSTA::setTargetReasoning(bool t) {
    cfg_target_reasoning_ = t;
}

void CBSTA::setMutexReasoning(bool m) {
    cfg_mutex_reasoning_ = m;
}

void CBSTA::setDisjointSplitting(bool d) {
    cfg_disjoint_splitting_ = d;
}

void CBSTA::setBypass(bool b) {
    cfg_bypass_ = b;
}

void CBSTA::setNodeLimit(int n) {
    cfg_node_limit_ = n;
}

void CBSTA::setSavingStats(bool s) {
    cfg_save_stats_ = s;
}

void CBSTA::computeCostMatrix() {
    cost_matrix_.resize(num_agents_, std::vector<int>(num_goals_, cbs_planner::ASSIGNMENT_INF));
    
    // For each agent, compute shortest path to each goal
    for (int agent = 0; agent < num_agents_; agent++) {
        // Create a temporary single-agent solver
        std::unique_ptr<SingleAgentSolver> solver;
        if (use_sipp_) {
            solver = std::make_unique<SIPP>(instance_, agent);
        } else {
            solver = std::make_unique<SpaceTimeAStar>(instance_, agent);
        }
        
        int start_loc = solver->start_location;
        
        for (int goal_idx = 0; goal_idx < num_goals_; goal_idx++) {
            if (!allowed_matrix_[agent][goal_idx]) {
                continue;  // Keep as infinity
            }
            
            int goal_loc = all_goals_[goal_idx];
            
            // Simple BFS/Manhattan distance for now
            // In a full implementation, we'd run A* to get actual shortest path
            int distance = instance_.getManhattanDistance(start_loc, goal_loc);
            
            // If we need exact distances, we could run the full pathfinder here
            // For optimal CBS-TA, the lower bound just needs to be admissible
            cost_matrix_[agent][goal_idx] = distance;
        }
    }
}

std::unique_ptr<CBS> CBSTA::createCBSForAssignment(const std::vector<int>& assignment) {
    // Clean up search engines from previous CBS solver
    if (cbs_solver_) {
        cbs_solver_->clearSearchEngines();
    }
    
    // Create search engines for agents with assigned goals
    std::vector<SingleAgentSolver*> search_engines(num_agents_);
    std::vector<ConstraintTable> constraints(num_agents_);
    std::vector<Path> initial_paths;
    
    for (int agent = 0; agent < num_agents_; agent++) {
        int goal_idx = assignment[agent];
        int goal_loc = all_goals_[goal_idx];
        
        // Create search engine for this agent
        if (use_sipp_) {
            search_engines[agent] = new SIPP(instance_, agent);
        } else {
            search_engines[agent] = new SpaceTimeAStar(instance_, agent);
        }
        
        // Override the goal location with the assigned goal.
        // IMPORTANT: this also recomputes my_heuristic so it points to the
        // assigned goal. Setting goal_location alone would leave the heuristic
        // (computed in the constructor for the instance's original goal) stale,
        // which produces sub-optimal paths and crashes MDD::buildMDD
        // (assert levels.back().size() == 1).
        search_engines[agent]->setGoalLocation(goal_loc);
        
        // Initialize constraint table with goal location
        constraints[agent] = ConstraintTable(instance_.num_of_cols, instance_.map_size);
        constraints[agent].goal_location = goal_loc;
    }
    
    // Create CBS solver with the configured search engines
    auto cbs = std::make_unique<CBS>(search_engines, constraints, initial_paths, screen_);
    
    cbs->setHeuristicType(cfg_heuristic_type_);
    cbs->setPrioritizeConflicts(cfg_PC_);
    cbs->setRectangleReasoning(cfg_rectangle_);
    cbs->setCorridorReasoning(cfg_corridor_);
    cbs->setTargetReasoning(cfg_target_reasoning_);
    cbs->setMutexReasoning(cfg_mutex_reasoning_);
    cbs->setDisjointSplitting(cfg_disjoint_splitting_);
    cbs->setBypass(cfg_bypass_);
    cbs->setNodeLimit(cfg_node_limit_);
    cbs->setSavingStats(cfg_save_stats_);
    
    return cbs;
}

bool CBSTA::solve(double time_limit, int cost_lowerbound, int cost_upperbound) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (screen_ > 0) {
        std::cout << "CBS-TA: " << num_agents_ << " agents, " 
                  << num_goals_ << " goals" << std::endl;
    }
    
    // Get the first (optimal) assignment
    auto first_assignment = task_assignment_->getFirstAssignment();
    if (first_assignment.empty()) {
        if (screen_ > 0) {
            std::cout << "CBS-TA: No feasible assignment exists" << std::endl;
        }
        solution_cost_ = -2;
        return false;
    }
    
    // Create first root node
    RootNode first_root;
    first_root.assignment = first_assignment;
    first_root.lower_bound = task_assignment_->getAssignmentCost(first_assignment);
    root_nodes_.push_back(first_root);
    
    int current_best_cost = MAX_COST;
    std::vector<int> best_assignment;
    std::vector<Path> best_paths;
    
    // Main CBS-TA loop: process root nodes in order of lower bound
    size_t root_idx = 0;
    while (root_idx < root_nodes_.size()) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed >= time_limit) {
            if (screen_ > 0) {
                std::cout << "CBS-TA: Timeout after " << elapsed << "s" << std::endl;
            }
            solution_cost_ = -1;
            break;
        }
        
        RootNode& current_root = root_nodes_[root_idx];
        
        // Pruning: if this root's lower bound >= current best, skip
        if (current_root.lower_bound >= current_best_cost) {
            if (screen_ > 1) {
                std::cout << "CBS-TA: Pruning root " << root_idx 
                          << " (LB=" << current_root.lower_bound 
                          << " >= best=" << current_best_cost << ")" << std::endl;
            }
            root_idx++;
            continue;
        }
        
        if (screen_ > 1) {
            std::cout << "CBS-TA: Processing root " << root_idx 
                      << " (LB=" << current_root.lower_bound << ")" << std::endl;
        }
        
        // Create CBS solver for this assignment
        cbs_solver_ = createCBSForAssignment(current_root.assignment);
        num_assignments_tried_++;
        
        // Debug: Log the assignment being tried
        if (screen_ > 0) {
            std::cout << "CBS-TA: Trying assignment [";
            for (size_t i = 0; i < current_root.assignment.size(); i++) {
                std::cout << current_root.assignment[i];
                if (i < current_root.assignment.size() - 1) std::cout << ",";
            }
            std::cout << "]" << std::endl;
        }
        
        // Configure CBS with all CBSH2-RTC features
        // (Caller can override via set* methods before solve())
        
        // Solve with remaining time
        double remaining_time = time_limit - elapsed;
        bool solved = cbs_solver_->solve(remaining_time, cost_lowerbound, 
                                         std::min(cost_upperbound, current_best_cost - 1));
        
        // Debug: Log CBS result
        if (screen_ > 0) {
            std::cout << "CBS-TA: CBS solve returned " << (solved ? "true" : "false")
                      << ", cost=" << cbs_solver_->solution_cost
                      << ", expanded=" << cbs_solver_->num_HL_expanded << std::endl;
        }
        
        // Accumulate statistics
        num_HL_expanded_ += cbs_solver_->num_HL_expanded;
        num_HL_generated_ += cbs_solver_->num_HL_generated;
        
        if (solved && cbs_solver_->solution_cost >= 0 && cbs_solver_->solution_cost < current_best_cost) {
            current_best_cost = cbs_solver_->solution_cost;
            best_assignment = current_root.assignment;
            
            // Extract paths from CBS solver
            best_paths = cbs_solver_->getPaths();
            solution_found_ = true;
            
            if (screen_ > 0) {
                std::cout << "CBS-TA: Found solution with cost " 
                          << current_best_cost << std::endl;
            }
        }
        
        // When expanding a root node, add the next assignment as a new root
        if (!current_root.expanded) {
            current_root.expanded = true;
            
            auto next_assignment = task_assignment_->getNextAssignment();
            if (next_assignment.has_value()) {
                RootNode new_root;
                new_root.assignment = *next_assignment;
                new_root.lower_bound = task_assignment_->getAssignmentCost(*next_assignment);
                
                // Insert in sorted order by lower bound
                auto insert_pos = std::lower_bound(
                    root_nodes_.begin() + root_idx + 1, root_nodes_.end(), new_root,
                    [](const RootNode& a, const RootNode& b) {
                        return a.lower_bound < b.lower_bound;
                    });
                root_nodes_.insert(insert_pos, new_root);
            }
        }
        
        root_idx++;
    }
    
    // Finalize results
    auto end_time = std::chrono::high_resolution_clock::now();
    runtime_ = std::chrono::duration<double>(end_time - start_time).count();
    
    if (solution_found_) {
        solution_cost_ = current_best_cost;
        optimal_assignment_ = best_assignment;
        solution_paths_ = best_paths;  // Store the solution paths
        
        if (screen_ > 0) {
            std::cout << "CBS-TA: Optimal solution cost = " << solution_cost_
                      << ", runtime = " << runtime_ << "s"
                      << ", HL nodes expanded = " << num_HL_expanded_ << std::endl;
        }
    }
    
    return solution_found_;
}

std::vector<int> CBSTA::getOptimalAssignment() const {
    return optimal_assignment_;
}

std::vector<Path> CBSTA::getPaths() const {
    return solution_paths_;
}

void CBSTA::printPaths() const {
    if (cbs_solver_) {
        // CBS has a printPaths method but it's private
        // We'll need to extract paths differently
        std::cout << "Optimal assignment:" << std::endl;
        for (size_t i = 0; i < optimal_assignment_.size(); i++) {
            std::cout << "  Agent " << i << " -> Goal " << optimal_assignment_[i]
                      << " (location " << all_goals_[optimal_assignment_[i]] << ")" << std::endl;
        }
    }
}

void CBSTA::saveResults(const std::string& fileName, const std::string& instanceName) const {
    std::ofstream file(fileName, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << fileName << std::endl;
        return;
    }
    
    file << instanceName << ","
         << runtime_ << ","
         << solution_cost_ << ","
         << num_HL_expanded_ << ","
         << num_HL_generated_ << ","
         << root_nodes_.size() << std::endl;
    
    file.close();
}

std::string CBSTA::getSolverName() const {
    return "CBS-TA";
}
