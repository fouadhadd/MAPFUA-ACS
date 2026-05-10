#include "scenario.hpp"


Scenario::Scenario(int map_rows, int map_cols, vector<vector<CellType>> &map,
                   vector<Location> &assigned_start_locations, vector<Location> &assigned_goal_locations,
                   vector<Location> &unassigned_start_locations) :
        map_rows(map_rows), map_cols(map_cols), map(map),
        num_of_assigned_agents(static_cast<int>(assigned_start_locations.size())),
        num_of_unassigned_agents(static_cast<int>(unassigned_start_locations.size())),
        assigned_start_locations(assigned_start_locations),
        assigned_goal_locations(assigned_goal_locations),
        unassigned_start_locations(unassigned_start_locations) {}

Scenario::~Scenario() {
    map.clear();
    assigned_start_locations.clear();
    assigned_goal_locations.clear();
    unassigned_start_locations.clear();
}

std::ostream &operator<<(std::ostream &os, const Scenario &s) {
    os << "Scenario: " << std::endl
       << "Map size: " << s.map_rows << "x" << s.map_cols << std::endl
       << "Num of assigned agents: " << s.num_of_assigned_agents << std::endl
       << "Assigned start locations: " << "[";
    for (int i = 0; i < s.num_of_assigned_agents; i++) {
        os << s.assigned_start_locations[i];
        if (i < s.num_of_assigned_agents - 1) os << ", ";
    }
    os << "]" << std::endl
       << "Assigned goal locations: " << "[";
    for (int i = 0; i < s.num_of_assigned_agents; i++) {
        os << s.assigned_goal_locations[i];
        if (i < s.num_of_assigned_agents - 1) os << ", ";
    }
    os << "]" << std::endl
       << "Num of unassigned agents: " << s.num_of_unassigned_agents << std::endl
       << "Unassigned start locations: " << "[";
    for (int i = 0; i < s.num_of_unassigned_agents; i++) {
        os << s.unassigned_start_locations[i];
        if (i < s.num_of_unassigned_agents - 1) os << ", ";
    }
    os << "]" << std::endl;
    return os;
}
