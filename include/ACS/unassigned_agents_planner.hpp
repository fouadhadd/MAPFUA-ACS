#ifndef MAPFUA_UNASSIGNED_AGENTS_PLANNER_HPP
#define MAPFUA_UNASSIGNED_AGENTS_PLANNER_HPP

#include "flow_node.hpp"
#include "../constraints/constraints.hpp"
#include "../ACS/capacity_conflict.hpp"
#include "ortools/graph/min_cost_flow.h"
#include "ortools/graph/graph.h"
#include <queue>
#include <list>

class UnassignedAgentsPlanner {
public:
    enum Result {
        SUCCESS,
        CONFLICT,
        INFEASIBLE
    };

private:
    int map_rows, map_cols;
    vector<vector<CellType>> map;
    int num_of_unassigned_agents;
    vector<Location> unassigned_agent_start_locations;
    bool verbose;

    typedef util::ReverseArcStaticGraph<> Graph;
    typedef Graph::NodeIndex NodeIndex;
    typedef Graph::ArcIndex ArcIndex;
    typedef operations_research::GenericMinCostFlow<Graph> MinCostFlow;
    typedef int32_t FlowQuantity;
    typedef int32_t CostValue;

    constexpr static const int dx[4] = {0, 1, 0, -1};
    constexpr static const int dy[4] = {1, 0, -1, 0};
    constexpr static const FlowQuantity CAP = 1;
    constexpr static const FlowQuantity BLOCKED_CAP = 0;
    constexpr static const CostValue NO_COST = 0;
    constexpr static const CostValue UNIT_COST = 1;

    vector<FlowNode> idx_to_node;
    unordered_map<FlowNode, NodeIndex> node_to_idx;
    FlowNode source_node;
    NodeIndex source_idx;
    FlowNode sink_node;
    NodeIndex sink_idx;
    int current_makespan;
    int max_makespan;
    int num_edges_before_sink;
    unordered_map<NodeIndex, unordered_map<NodeIndex, ArcIndex>> arc_map;

    vector<NodeIndex> arc_tail;
    vector<NodeIndex> arc_head;
    vector<CostValue> arc_cost;
    vector<FlowQuantity> arc_cap;
    vector<ArcIndex> arc_permutation;
    vector<FlowQuantity> arc_flow;
    unordered_map<NodeIndex, NodeIndex> flow;
    CostValue optimal_cost;
    FlowQuantity maximum_flow;

    vector<Location> reached_locations;
    unordered_set<Location> new_reached_locations;
    unordered_set<Location> recently_reached_locations;
    vector<std::pair<Location, Location>> map_edges;
    bool reached_all = false;

    unordered_map<ArcIndex, FlowQuantity> orig_edge_caps;
    unordered_map<NodeIndex, NodeIndex> commit_edges_d_to_s;
    unordered_set<ArcIndex> deleted_arcs;

    unordered_map<int, array<int, 3>> makespan_graph_params;
    unordered_map<int, vector<ArcIndex>> makespan_to_arc_permutation;

    struct BlockData {
        int agent_id;
        Location loc1;
        Location loc2;
        int t;
    };

    unordered_map<ArcIndex, BlockData> blocked_arcs_info;

    NodeIndex get_node_idx(const FlowNode &node);
    ArcIndex get_arc_idx(NodeIndex tail, NodeIndex head);
    void add_arc(NodeIndex tail, NodeIndex head, CostValue cost);
    void add_arc_to_sink(NodeIndex tail);
    void update_map_edges();
    void update_graph(int makespan);

    void block_assigned_agents_paths(const vector<shared_ptr<TimedPath>> &assigned_agents_paths);
    ArcIndex update_single_edge_cap(NodeIndex n1, NodeIndex n2, FlowQuantity cap);
    Result solve_flow();
    Result extract_capacity_conflict();
    ArcIndex PermutedArc(ArcIndex arc) const;
    void print_flow();

public:
    vector<shared_ptr<TimedPath>> unassigned_agent_paths;
    std::vector<shared_ptr<CapacityConflict>> capacity_conflicts;

    UnassignedAgentsPlanner(int map_rows, int map_cols, const vector<vector<CellType>> &map, vector<Location>
                                &unassigned_agent_start_locations, bool verbose);

    Result find_paths(const vector<shared_ptr<TimedPath>>& assigned_agents_paths, int makespan);

    Result extract_unassigned_agent_paths(const vector<shared_ptr<TimedPath>> &assigned_agents_paths);
};


#endif //MAPFUA_UNASSIGNED_AGENTS_PLANNER_HPP
