/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, March 2021
*/

#pragma once

#include <vector>
#include <string>
#include <memory>

// Forward declarations to minimize dependencies
class CBS;
class Instance;

namespace cbs_planner {

/**
 * @brief Main interface for CBS-based multi-agent path planning
 * 
 * This class provides a simplified interface to the CBS (Conflict-Based Search) 
 * algorithm for multi-agent path finding. It handles the complexity of the 
 * underlying CBS implementation and provides thread-safe access.
 * 
 * Thread-safety: This class is NOT thread-safe. Create separate instances 
 * for concurrent use.
 */
class CBSPlanner {
public:
    /**
     * @brief Represents a 2D coordinate (row, col)
     */
    using Coordinate = std::pair<int, int>;
    
    /**
     * @brief Represents a path as a sequence of coordinates
     */
    using Path = std::vector<Coordinate>;
    
    /**
     * @brief Represents multiple agent paths
     */
    using Paths = std::vector<Path>;

    /**
     * @brief Configuration parameters for the CBS planner
     */
    struct Config {
        double time_limit = 60.0;              // Time limit in seconds
        int node_limit = 100000;               // Maximum number of nodes to expand
        bool prioritize_conflicts = true;      // Use conflict prioritization
        bool bypass = true;                    // Use bypass reasoning
        bool target_reasoning = true;          // Use target reasoning
        bool mutex_reasoning = false;          // Use mutex reasoning
        bool disjoint_splitting = false;       // Use disjoint splitting
        bool rectangle_reasoning = false;      // Use rectangle reasoning
        bool corridor_reasoning = false;       // Use corridor reasoning
        bool use_sipp = false;                 // Use SIPP for low-level search
    };

    /**
     * @brief Result of a planning operation
     */
    struct Result {
        bool success = false;                  // Whether a solution was found
        Paths paths;                          // The computed paths
        int solution_cost = -1;               // Total cost of the solution
        double runtime = 0.0;                 // Runtime in seconds
        int num_expanded = 0;                 // Number of high-level nodes expanded
        int num_generated = 0;                // Number of high-level nodes generated
        std::string error_message;            // Error message if planning failed
    };

public:
    /**
     * @brief Constructor
     */
    CBSPlanner();
    
    /**
     * @brief Destructor
     */
    ~CBSPlanner();

    /**
     * @brief Plan paths for multiple agents
     * 
     * @param map_file Path to the map file (.pgm, .map, or custom format)
     * @param starts Starting positions for each agent
     * @param goals Goal positions for each agent
     * @param config Configuration parameters (optional)
     * @return Result containing paths and planning statistics
     */
    Result planPaths(const std::string& map_file,
                    const std::vector<Coordinate>& starts,
                    const std::vector<Coordinate>& goals,
                    const Config& config = getDefaultConfig());

    /**
     * @brief Plan paths using a pre-loaded map
     * 
     * @param map_data 2D grid map (true = obstacle, false = free)
     * @param starts Starting positions for each agent
     * @param goals Goal positions for each agent
     * @param config Configuration parameters (optional)
     * @return Result containing paths and planning statistics
     */
    Result planPaths(const std::vector<std::vector<bool>>& map_data,
                    const std::vector<Coordinate>& starts,
                    const std::vector<Coordinate>& goals,
                    const Config& config = getDefaultConfig());

    /**
     * @brief Plan paths using scenario file (.scen format)
     * 
     * This method automatically parses the .scen file to extract start and goal
     * positions for all agents, then performs path planning using the specified map.
     * 
     * @param map_file Path to the map file (.pgm, .map, or custom format)
     * @param scenario_file Path to the scenario file (.scen format)
     * @param config Configuration parameters (optional)
     * @return Result containing paths and planning statistics
     */
    Result planPaths(const std::string& map_file,
                    const std::string& scenario_file,
                    const Config& config = getDefaultConfig());

    /**
     * @brief Plan paths using pre-loaded map and scenario file (.scen format)
     * 
     * This method automatically parses the .scen file to extract start and goal
     * positions for all agents, then performs path planning using the provided map data.
     * 
     * @param map_data 2D grid map (true = obstacle, false = free)
     * @param scenario_file Path to the scenario file (.scen format)
     * @param config Configuration parameters (optional)
     * @return Result containing paths and planning statistics
     */
    Result planPaths(const std::vector<std::vector<bool>>& map_data,
                    const std::string& scenario_file,
                    const Config& config = getDefaultConfig());

    /**
     * @brief Parse a scenario file to extract starts and goals
     * 
     * @param scenario_file Path to the scenario file (.scen format)
     * @param starts Output vector for start coordinates
     * @param goals Output vector for goal coordinates
     * @return true if parsing was successful, false otherwise
     */
    static bool parseScenarioFile(const std::string& scenario_file,
                                std::vector<Coordinate>& starts,
                                std::vector<Coordinate>& goals);

    /**
     * @brief Generate random agents on an existing map and save to .scen file
     * 
     * @param map_file Path to the existing map file
     * @param num_agents Number of agents to generate
     * @param output_file Path to save the generated scenario (.scen)
     * @return true if generation was successful
     */
    bool generateRandomScenario(const std::string& map_file, 
                              int num_agents, 
                              const std::string& output_file);

    /**
     * @brief Generate random scenario with fixed start locations
     * 
     * @param map_file Path to the existing map file
     * @param starts Vector of start coordinates (row, col)
     * @param output_file Path to save the generated scenario (.scen)
     * @return true if generation was successful
     */
    bool generateRandomScenario(const std::string& map_file,
                              const std::vector<Coordinate>& starts,
                              const std::string& output_file);

    /**
     * @brief Generate random map and agents, saving to .pgm and .scen files
     * 
     * @param rows Map height
     * @param cols Map width
     * @param obstacles Number of obstacles
     * @param num_agents Number of agents to generate
     * @param output_map_file Path to save the generated map (.pgm)
     * @param output_scen_file Path to save the generated scenario (.scen)
     * @return true if generation was successful
     */
    bool generateRandomInstance(int rows, int cols, int obstacles, 
                              int num_agents, 
                              const std::string& output_map_file, 
                              const std::string& output_scen_file);


    /**
     * @brief Get the default configuration
     */
    static Config getDefaultConfig();

    /**
     * @brief Check if a coordinate is valid for the given map dimensions
     */
    static bool isValidCoordinate(const Coordinate& coord, int rows, int cols);

    /**
     * @brief Convert paths to a human-readable string format
     */
    static std::string pathsToString(const Paths& paths);

private:
    // Private implementation to hide CBS internals
    class Impl;
    std::unique_ptr<Impl> pImpl;
    
    // Disable copy constructor and assignment operator
    CBSPlanner(const CBSPlanner&) = delete;
    CBSPlanner& operator=(const CBSPlanner&) = delete;
};

} // namespace cbs_planner
