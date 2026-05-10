#ifndef MAPFUA_SCENARIO_HPP
#define MAPFUA_SCENARIO_HPP

#include "common.hpp"


class Scenario {
public:
    int map_rows;
    int map_cols;
    vector<vector<CellType>> map;

    int num_of_assigned_agents;
    int num_of_unassigned_agents;

    vector<Location> assigned_start_locations;
    vector<Location> assigned_goal_locations;
    vector<Location> unassigned_start_locations;

    Scenario(int map_rows, int map_cols, vector<vector<CellType>> &map,
             vector<Location> &assigned_start_locations, vector<Location> &assigned_goal_locations,
             vector<Location> &unassigned_start_locations);
    ~Scenario();
    friend std::ostream &operator<<(std::ostream &os, const Scenario &s);
};


#endif //MAPFUA_SCENARIO_HPP
