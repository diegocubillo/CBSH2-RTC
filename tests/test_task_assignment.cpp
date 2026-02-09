/*
 * test_task_assignment.cpp - Unit tests for TaskAssignment module
 */

#include <gtest/gtest.h>
#include "TaskAssignment.h"

using namespace cbs_planner;

class TaskAssignmentTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

// Test 1: Simple 3x3 assignment (square matrix)
TEST_F(TaskAssignmentTest, HungarianSimple3x3) {
    // Cost matrix: 3 agents, 3 goals
    std::vector<std::vector<int>> costs = {
        {10, 5, 13},
        {3, 15, 8},
        {12, 6, 7}
    };
    std::vector<std::vector<bool>> allowed(3, std::vector<bool>(3, true));
    
    TaskAssignment ta(costs, allowed);
    auto assignment = ta.getFirstAssignment();
    
    ASSERT_EQ(assignment.size(), 3);
    
    // Verify all agents are assigned to different goals
    std::set<int> assigned_goals(assignment.begin(), assignment.end());
    EXPECT_EQ(assigned_goals.size(), 3);
    
    // Optimal assignment should have cost 15 (5 + 3 + 7)
    // Agent 0 -> Goal 1 (cost 5)
    // Agent 1 -> Goal 0 (cost 3)
    // Agent 2 -> Goal 2 (cost 7)
    int cost = ta.getAssignmentCost(assignment);
    EXPECT_EQ(cost, 15);
}

// Test 2: Rectangular matrix (more goals than agents)
TEST_F(TaskAssignmentTest, HungarianRectangular) {
    // 2 agents, 4 goals
    std::vector<std::vector<int>> costs = {
        {10, 5, 3, 8},
        {7, 2, 6, 4}
    };
    std::vector<std::vector<bool>> allowed(2, std::vector<bool>(4, true));
    
    TaskAssignment ta(costs, allowed);
    auto assignment = ta.getFirstAssignment();
    
    ASSERT_EQ(assignment.size(), 2);
    
    // Agents should be assigned to different goals
    EXPECT_NE(assignment[0], assignment[1]);
    
    // Optimal: Agent 0 -> Goal 2 (cost 3), Agent 1 -> Goal 1 (cost 2) = 5
    int cost = ta.getAssignmentCost(assignment);
    EXPECT_EQ(cost, 5);
}

// Test 3: K-best enumeration
TEST_F(TaskAssignmentTest, KBestEnumeration) {
    std::vector<std::vector<int>> costs = {
        {1, 2},
        {3, 4}
    };
    std::vector<std::vector<bool>> allowed(2, std::vector<bool>(2, true));
    
    TaskAssignment ta(costs, allowed);
    
    // First assignment: optimal
    auto first = ta.getFirstAssignment();
    int first_cost = ta.getAssignmentCost(first);
    
    // Second assignment: next best
    auto second = ta.getNextAssignment();
    ASSERT_TRUE(second.has_value());
    int second_cost = ta.getAssignmentCost(*second);
    
    // Second cost should be >= first cost
    EXPECT_GE(second_cost, first_cost);
    
    // Assignments should be different
    EXPECT_NE(first, *second);
    
    // For 2x2, there are only 2 possible assignments
    auto third = ta.getNextAssignment();
    EXPECT_FALSE(third.has_value());
}

// Test 4: Constrained assignment (some forbidden)
TEST_F(TaskAssignmentTest, ConstrainedAssignment) {
    std::vector<std::vector<int>> costs = {
        {1, 10},
        {10, 1}
    };
    // Agent 0 can only go to goal 1, Agent 1 can only go to goal 0
    std::vector<std::vector<bool>> allowed = {
        {false, true},
        {true, false}
    };
    
    TaskAssignment ta(costs, allowed);
    auto assignment = ta.getFirstAssignment();
    
    ASSERT_EQ(assignment.size(), 2);
    EXPECT_EQ(assignment[0], 1);  // Agent 0 -> Goal 1
    EXPECT_EQ(assignment[1], 0);  // Agent 1 -> Goal 0
    
    // Cost = 10 + 10 = 20
    EXPECT_EQ(ta.getAssignmentCost(assignment), 20);
}

// Test 5: No feasible assignment
TEST_F(TaskAssignmentTest, NoFeasibleAssignment) {
    std::vector<std::vector<int>> costs = {
        {1, 2},
        {3, 4}
    };
    // Both agents can only go to goal 0
    std::vector<std::vector<bool>> allowed = {
        {true, false},
        {true, false}
    };
    
    TaskAssignment ta(costs, allowed);
    auto assignment = ta.getFirstAssignment();
    
    // Should return empty (infeasible)
    EXPECT_TRUE(assignment.empty());
}

// Test 6: Single agent
TEST_F(TaskAssignmentTest, SingleAgent) {
    std::vector<std::vector<int>> costs = {
        {5, 3, 7}
    };
    std::vector<std::vector<bool>> allowed(1, std::vector<bool>(3, true));
    
    TaskAssignment ta(costs, allowed);
    auto assignment = ta.getFirstAssignment();
    
    ASSERT_EQ(assignment.size(), 1);
    EXPECT_EQ(assignment[0], 1);  // Cheapest is goal 1 with cost 3
    EXPECT_EQ(ta.getAssignmentCost(assignment), 3);
}

// Test 7: Validate assignment
TEST_F(TaskAssignmentTest, ValidateAssignment) {
    std::vector<std::vector<int>> costs = {
        {1, 2},
        {3, 4}
    };
    std::vector<std::vector<bool>> allowed = {
        {true, true},
        {true, false}  // Agent 1 cannot go to goal 1
    };
    
    TaskAssignment ta(costs, allowed);
    
    // Valid assignment
    Assignment valid = {0, 0};  // Both to goal 0 - invalid (collision)
    Assignment valid2 = {1, 0};  // Agent 0 to goal 1, Agent 1 to goal 0 - valid
    
    EXPECT_FALSE(ta.isValidAssignment(valid));  // Same goal
    EXPECT_TRUE(ta.isValidAssignment(valid2));
    
    // Invalid: forbidden pair
    Assignment forbidden = {0, 1};  // Agent 1 to goal 1 is forbidden
    EXPECT_FALSE(ta.isValidAssignment(forbidden));
}

// Test 8: Cost enumeration order
TEST_F(TaskAssignmentTest, EnumerationOrder) {
    std::vector<std::vector<int>> costs = {
        {1, 5, 10},
        {2, 3, 8},
        {4, 6, 7}
    };
    std::vector<std::vector<bool>> allowed(3, std::vector<bool>(3, true));
    
    TaskAssignment ta(costs, allowed);
    
    std::vector<int> costs_sequence;
    
    auto first = ta.getFirstAssignment();
    costs_sequence.push_back(ta.getAssignmentCost(first));
    
    for (int i = 0; i < 5; i++) {  // Get next 5 assignments
        auto next = ta.getNextAssignment();
        if (!next.has_value()) break;
        costs_sequence.push_back(ta.getAssignmentCost(*next));
    }
    
    // Verify costs are non-decreasing
    for (size_t i = 1; i < costs_sequence.size(); i++) {
        EXPECT_GE(costs_sequence[i], costs_sequence[i-1])
            << "Cost sequence should be non-decreasing";
    }
}
