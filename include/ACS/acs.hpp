#ifndef MAPFUA_ACS_HPP
#define MAPFUA_ACS_HPP

#include "../scenario.hpp"
#include "acs_node.hpp"
#include "../AStar/astar.hpp"
#include "unassigned_agents_planner.hpp"

class ACS {
private:
    int num_of_assigned;
    int num_of_unassigned;
    bool verbose;
    int iterations = 0;
    int node_count = 0;

    vector<int> assigned_start_ts;

    vector<AStar> solvers;
    UnassignedAgentsPlanner unassigned_planner;

    typedef boost::heap::d_ary_heap<shared_ptr<ACSNode>, boost::heap::arity<2>, boost::heap::mutable_<true>,
            boost::heap::compare<ACSNodeCompare>> OpenList;

public:
    ACS(int map_rows, int map_cols, vector<vector<CellType>> &map,
        vector<Location> &agent_start_locations,
        vector<Location> &delivery_locations, // End goals for assigned agents
        vector<Location> &unassigned_start_locations,
        bool verbose);

    ACS(Scenario &scenario, bool verbose);

    // The main ACS solve loop: Phase 1 (CBS for assigned) + Phase 2 (Flow for unassigned)
    shared_ptr<Plan> solve(int time_limit, bool& time_limit_exceeded);
    shared_ptr<Plan> solve(int time_limit = 60);

private:
    shared_ptr<ACSNode> get_root_node();

    shared_ptr<TimedPath> plan_single_assigned_path(int agent_id, shared_ptr<Constraints> constraints);

    // Standard CBS branching: Creates child nodes to resolve a collision
    std::pair<shared_ptr<ACSNode>, shared_ptr<ACSNode>>
    resolve_assigned_conflict(const shared_ptr<ACSNode>& node, const shared_ptr<Conflict>& conflict);
};

#endif //MAPFUA_ACS_HPP