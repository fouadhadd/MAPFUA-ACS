#include "unassigned_agents_planner.hpp"
#include "ortools/graph/max_flow.h"


UnassignedAgentsPlanner::UnassignedAgentsPlanner(int map_rows, int map_cols, const vector<vector<CellType>> &map,
                         vector<Location> &unassigned_agent_start_locations, bool verbose) :
        map_rows(map_rows), map_cols(map_cols), map(map), num_of_unassigned_agents(static_cast<int>(unassigned_agent_start_locations.size())),
        unassigned_agent_start_locations(unassigned_agent_start_locations), verbose(verbose),
        source_node(FlowNode::SOURCE, -1), source_idx(get_node_idx(source_node)),
        sink_node(FlowNode::SINK, -1), sink_idx(get_node_idx(sink_node)),
        current_makespan(0), max_makespan(0), num_edges_before_sink(0),
        optimal_cost(0), maximum_flow(0),
        unassigned_agent_paths(num_of_unassigned_agents, shared_ptr<TimedPath>()) {
    reached_locations.reserve(map_rows * map_cols);
    map_edges.reserve(map_rows * map_cols * 2 - map_rows - map_cols);
    new_reached_locations.reserve(num_of_unassigned_agents*4);
    for (int i = 0; i < num_of_unassigned_agents; ++i) {
        NodeIndex idx = get_node_idx(FlowNode(FlowNode::Type::OUT, 0, unassigned_agent_start_locations[i]));
        add_arc(source_idx, idx, NO_COST);
        reached_locations.emplace_back(unassigned_agent_start_locations[i]);
        new_reached_locations.insert(unassigned_agent_start_locations[i]);
    }
}


UnassignedAgentsPlanner::NodeIndex UnassignedAgentsPlanner::get_node_idx(const FlowNode &node) {
    auto it = node_to_idx.find(node);
    if (it == node_to_idx.end()) {
        const auto idx = static_cast<NodeIndex>(idx_to_node.size());
        idx_to_node.push_back(node);
        node_to_idx.emplace(node, idx);
        return idx;
    }
    return it->second;
}

UnassignedAgentsPlanner::ArcIndex UnassignedAgentsPlanner::get_arc_idx(NodeIndex tail, NodeIndex head) {
    return arc_map.at(tail).at(head);
}

void UnassignedAgentsPlanner::add_arc(NodeIndex tail, NodeIndex head, CostValue cost) {
    const auto idx = static_cast<ArcIndex>(arc_tail.size());
    arc_map[tail][head] = idx;
    arc_tail.push_back(tail);
    arc_head.push_back(head);
    arc_cost.push_back(cost);
    arc_cap.push_back(CAP);
    num_edges_before_sink++;
}

void UnassignedAgentsPlanner::add_arc_to_sink(NodeIndex tail) {
    arc_tail.push_back(tail);
    arc_head.push_back(sink_idx);
    arc_cost.push_back(NO_COST);
    arc_cap.push_back(CAP);
}


void UnassignedAgentsPlanner::update_map_edges() {
    if (reached_all) {
        return;
    }

    reached_all = true;

    bool b2;
    unordered_set<Location> next_reached_locations(new_reached_locations.size() * 4);
    unordered_set<Location> done_new(new_reached_locations.size());
    for (const Location& loc: new_reached_locations) {
        for (int i = 0; i < 4; i++) {
            Location new_loc(loc.x + dx[i], loc.y + dy[i]);
            if (new_loc.x < 0 || new_loc.x >= map_rows || new_loc.y < 0 || new_loc.y >= map_cols ||
                map[new_loc.x][new_loc.y] == CellType::BLOCKED) {
                continue;
            }
            std::pair edge = (loc < new_loc) ? std::make_pair(loc, new_loc) : std::make_pair(new_loc, loc);

            if (recently_reached_locations.count(new_loc) || done_new.count(new_loc)) {
                assert(std::any_of(map_edges.begin(), map_edges.end(),
                                   [edge](const std::pair<Location, Location> &e) { return e == edge; }));
                assert(std::any_of(reached_locations.begin(), reached_locations.end(),
                                   [new_loc](const Location loc) { return loc == new_loc; }));
                continue;
            }
            if (next_reached_locations.count(new_loc) || new_reached_locations.count(new_loc)) {
                assert(std::all_of(map_edges.begin(), map_edges.end(),
                                   [edge](const std::pair<Location, Location> &e) { return e != edge; }));
                assert(std::any_of(reached_locations.begin(), reached_locations.end(),
                                   [new_loc](const Location loc) { return loc == new_loc; }));
                map_edges.emplace_back(edge);
                continue;
            }

            assert(std::all_of(map_edges.begin(), map_edges.end(),
                               [edge](const std::pair<Location, Location> &e) { return e != edge; }));
            map_edges.emplace_back(edge);
            std::tie(std::ignore, b2) = next_reached_locations.emplace(new_loc);

            assert(b2);
            reached_all = false;
            assert(std::all_of(reached_locations.begin(), reached_locations.end(),
                               [new_loc](const Location loc) { return loc != new_loc; }));
            reached_locations.emplace_back(new_loc);
        }
        done_new.emplace(loc);
    }

    if (reached_all) {
        reached_all = true;
        recently_reached_locations.clear();
        new_reached_locations.clear();
        return;
    }

    recently_reached_locations = std::move(new_reached_locations);
    new_reached_locations = std::move(next_reached_locations);
}

void UnassignedAgentsPlanner::update_graph(int makespan) {
    if (current_makespan == makespan) {
        return;
    } else if (verbose) {
        std::cout << "Updating graph from makespan " << current_makespan << " to " << makespan << std::endl;
    }

    // remove all edges from previous makespan to sink
    arc_tail.resize(num_edges_before_sink);
    arc_head.resize(num_edges_before_sink);
    arc_cost.resize(num_edges_before_sink);
    arc_cap.resize(num_edges_before_sink);

    for (int t = max_makespan + 1; t <= makespan; t++) {
        // v_t-1_out -> v_t_in
        for (auto &loc : reached_locations) {
            NodeIndex v_out = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t - 1, loc));
            NodeIndex v_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc));
            this->add_arc(v_out, v_in, NO_COST);
        }

        // edge gadgets
        update_map_edges();
        for (const auto& [loc1, loc2] : map_edges) {
            auto v_1_out_it = node_to_idx.find(FlowNode(FlowNode::Type::OUT, t - 1, loc1));
            auto v_2_out_it = node_to_idx.find(FlowNode(FlowNode::Type::OUT, t - 1, loc2));

            NodeIndex edge_in = get_node_idx(FlowNode(FlowNode::Type::EDGE_IN, t, loc1, loc2));

            assert(v_1_out_it != node_to_idx.end() || v_2_out_it != node_to_idx.end());
            if (v_1_out_it != node_to_idx.end()) {
                add_arc(v_1_out_it->second, edge_in, NO_COST);
            }
            if (v_2_out_it != node_to_idx.end()) {
                add_arc(v_2_out_it->second, edge_in, NO_COST);
            }

            NodeIndex edge_out = get_node_idx(FlowNode(FlowNode::Type::EDGE_OUT, t, loc1, loc2));
            this->add_arc(edge_in, edge_out, UNIT_COST);

            NodeIndex v_1_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc1));
            NodeIndex v_2_in = get_node_idx(FlowNode(FlowNode::Type::IN, t, loc2));

            this->add_arc(edge_out, v_1_in, NO_COST);
            this->add_arc(edge_out, v_2_in, NO_COST);
        }

        // v_t_in -> v_t_out
        for (auto &loc : reached_locations) {
            NodeIndex v_in = node_to_idx.at(FlowNode(FlowNode::Type::IN, t, loc));
            NodeIndex v_out = get_node_idx(FlowNode(FlowNode::Type::OUT, t, loc));
            this->add_arc(v_in, v_out, NO_COST);
        }
    }

    // v_T_out -> sink
    for (auto &loc : reached_locations) {
        FlowNode f_node(FlowNode::Type::OUT, makespan, loc);
        auto it = node_to_idx.find(f_node);
        if (it == node_to_idx.end()) continue;
        this->add_arc_to_sink(it->second);
    }

    current_makespan = makespan;
    max_makespan = std::max(max_makespan, makespan);
}

void UnassignedAgentsPlanner::block_assigned_agents_paths(const vector<shared_ptr<TimedPath>> &assigned_agents_paths) {
    for (auto &path : assigned_agents_paths) {
        if (!path || path->empty()) continue;

        int last_t = std::min(current_makespan, path->back().time);
        for (int t = path->front().time; t < last_t; t++) {
            Location loc1 = path->get_location_at_time(t);
            Location loc2 = path->get_location_at_time(t + 1);

            // 1. Block the vertex (Prevents standing on or crossing the occupied tile)
            auto v_out_it = node_to_idx.find(FlowNode(FlowNode::Type::OUT, t, loc2));
            auto v_in_it = node_to_idx.find(FlowNode(FlowNode::Type::IN, t + 1, loc2));
            if (v_out_it != node_to_idx.end() && v_in_it != node_to_idx.end()) {
                update_single_edge_cap(v_out_it->second, v_in_it->second, BLOCKED_CAP);
            }

            // 2. Block the edge (Prevents head-on edge collisions or swapping places)
            if (loc1 != loc2) {
                std::pair edge = (loc1 < loc2) ? std::make_pair(loc1, loc2) : std::make_pair(loc2, loc1);

                auto edge_in_it = node_to_idx.find(FlowNode(FlowNode::Type::EDGE_IN, t + 1, edge.first, edge.second));
                auto edge_out_it = node_to_idx.find(FlowNode(FlowNode::Type::EDGE_OUT, t + 1, edge.first, edge.second));

                if (edge_in_it != node_to_idx.end() && edge_out_it != node_to_idx.end()) {
                    update_single_edge_cap(edge_in_it->second, edge_out_it->second, BLOCKED_CAP);
                }
            }
        }
    }
}

void UnassignedAgentsPlanner::update_single_edge_cap(NodeIndex tail, NodeIndex head, FlowQuantity cap) {
    // 1. Safely look up the arc without using .at() to prevent out-of-range crashes
    auto it = arc_map.find(tail);
    if (it != arc_map.end()) {
        auto it2 = it->second.find(head);
        if (it2 != it->second.end()) {
            ArcIndex arc = it2->second;

            // 2. Critical bounds check to stop 0xC0000005 segmentation faults
            if (arc >= 0 && arc < static_cast<ArcIndex>(arc_cap.size())) {

                // 3. Save the old CAPACITY
                orig_edge_caps.emplace(arc, arc_cap[arc]);

                // 4. Apply the new hard block
                arc_cap[arc] = cap;

                // 5. CRITICAL: Track it so the Cleanup Crew can fix it later!
                deleted_arcs.emplace(arc);
            }
        }
    }
}

UnassignedAgentsPlanner::Result UnassignedAgentsPlanner::find_paths(const vector<shared_ptr<TimedPath>>
                                                                        &assigned_agents_paths, int makespan) {
    if (num_of_unassigned_agents == 0) return SUCCESS;

    for (auto arc : deleted_arcs) {
        if (arc >= 0 && arc < static_cast<ArcIndex>(arc_cap.size())) {
            arc_cap[arc] = orig_edge_caps[arc];
        }
    }
    deleted_arcs.clear();
    orig_edge_caps.clear();

    update_graph(makespan);
    block_assigned_agents_paths(assigned_agents_paths);

    auto result = this->solve_flow();
    if (result != SUCCESS) {
        return result;
    }

    return this->extract_agent_paths();
}

UnassignedAgentsPlanner::Result UnassignedAgentsPlanner::solve_flow() {
    optimal_cost = 0;
    maximum_flow = 0;
    arc_flow.clear(); flow.clear();
    const auto num_nodes = static_cast<NodeIndex>(idx_to_node.size());
    const auto num_arcs = static_cast<ArcIndex>(arc_tail.size());
    assert(num_nodes > 0);

    if (arc_tail.size() != num_arcs || arc_cost.size() != num_arcs || arc_cap.size() != num_arcs) {
        return INFEASIBLE;
    }

    // Feasibility checking, and possible supply/demand adjustment, is done by:
    // 1. Creating a new source and sink node.
    // 2. Taking all nodes that have a non-zero supply or demand and connecting them to the source or sink
    //    respectively. The arc thus added has a capacity of the supply or demand.
    // 3. Computing the max flow between the new source and sink.
    // 4. Checking that the max flow is equal to the total supply/demand (and returning INFEASIBLE if it isn't).
    // 5. Running min-cost_type max-flow on this augmented graph, using the max flow computed in step 3 as the supply of
    //    the source and demand of the sink.
    const ArcIndex augmented_num_arcs = num_arcs + 2;
    const NodeIndex total_source = num_nodes;
    const NodeIndex total_sink = num_nodes + 1;
    const NodeIndex augmented_num_nodes = num_nodes + 2;

    Graph graph(augmented_num_nodes, augmented_num_arcs);

    for (ArcIndex arc = 0; arc < num_arcs; ++arc) {
        graph.AddArc(arc_tail.at(arc), arc_head.at(arc));
    }

    ArcIndex structural_source_arc = graph.AddArc(total_source, source_idx);
    ArcIndex structural_sink_arc = graph.AddArc(sink_idx, total_sink);

    graph.Build(&arc_permutation);

    {
        operations_research::GenericMaxFlow<Graph> max_flow(&graph, total_source, total_sink);
        ArcIndex arc;
        for (arc = 0; arc < num_arcs; ++arc) {
            max_flow.SetArcCapacity(PermutedArc(arc), arc_cap.at(arc));
        }
        for (; arc < augmented_num_arcs - 2; ++arc) {
            max_flow.SetArcCapacity(PermutedArc(arc), CAP);
        }

        max_flow.SetArcCapacity(PermutedArc(arc++), num_of_unassigned_agents);
        max_flow.SetArcCapacity(PermutedArc(arc++), num_of_unassigned_agents);

        assert(arc == augmented_num_arcs);
        if (!max_flow.Solve()) {
            throw std::runtime_error("Max flow could not be computed.");
        }
        maximum_flow = static_cast<FlowQuantity>(max_flow.GetOptimalFlow());
    }

    if (maximum_flow != num_of_unassigned_agents) {
        return INFEASIBLE;
    }

    std::cout << maximum_flow << std::endl;

    size_t num = idx_to_node.size();
    for (size_t i = 0; i < arc_tail.size(); ++i) {
        if (arc_tail[i] >= num || arc_head[i] >= num) {
            std::cerr << "DEBUG: Poison index found at arc " << i
                      << ": " << arc_tail[i] << " -> " << arc_head[i]
                      << " (Num nodes: " << num << ")" << std::endl;
            // If this prints, your get_node_idx() logic is creating arcs for nodes
            // that aren't in the current node index map.
        }
    }

    // --- 1. USE SIMPLE MIN COST FLOW INSTEAD ---
    operations_research::SimpleMinCostFlow min_cost_flow;

    // 2. Add all map edges directly to the solver using your raw arrays.
    // NO PermutedArc needed! The order you add them is the order they are stored.
    ArcIndex arc;
    for (arc = 0; arc < num_arcs; ++arc) {
        min_cost_flow.AddArcWithCapacityAndUnitCost(arc_tail[arc], arc_head[arc], arc_cap[arc], arc_cost[arc]);
    }

    // 3. Add the structural source and sink edges
    min_cost_flow.AddArcWithCapacityAndUnitCost(total_source, source_idx, num_of_unassigned_agents, NO_COST);
    min_cost_flow.AddArcWithCapacityAndUnitCost(sink_idx, total_sink, num_of_unassigned_agents, NO_COST);

    // 4. Set Supplies
    min_cost_flow.SetNodeSupply(total_source, maximum_flow);
    min_cost_flow.SetNodeSupply(total_sink, -maximum_flow);

    // 5. Solve and Extract cleanly
    arc_flow.resize(num_arcs);
    flow.clear();

    // SimpleMinCostFlow handles all feasibility checks internally and safely.
    if (min_cost_flow.Solve() == operations_research::MinCostFlowBase::OPTIMAL) {
        optimal_cost = static_cast<CostValue>(min_cost_flow.OptimalCost());

        for (arc = 0; arc < num_arcs; ++arc) {
            // Because we didn't permute, mcf.Flow(arc) perfectly matches arc_tail[arc]
            arc_flow[arc] = static_cast<FlowQuantity>(min_cost_flow.Flow(arc));
            if (arc_flow[arc] > 0) {
                if (verbose){
                    std::cout << "f " << idx_to_node.at(arc_tail[arc]) << " -> "
                              << idx_to_node.at(arc_head[arc]) << " " << arc_flow[arc] << std::endl;
                }
                flow[arc_tail[arc]] = arc_head[arc];
            }
        }
    } else {
        return INFEASIBLE;
    }

    if (verbose) {
        this->print_flow();
    }

    std::cout << optimal_cost << std::endl;

    return SUCCESS;
}

UnassignedAgentsPlanner::ArcIndex UnassignedAgentsPlanner::PermutedArc(ArcIndex arc) const {
    return arc < static_cast<int>(arc_permutation.size()) ? arc_permutation[arc] : arc;
}

UnassignedAgentsPlanner::Result UnassignedAgentsPlanner::extract_agent_paths() {
    for (int a_i = 0; a_i < num_of_unassigned_agents; ++a_i) {
        unassigned_agent_paths[a_i] = std::make_shared<TimedPath>();
        unassigned_agent_paths[a_i]->push_back(State(0, unassigned_agent_start_locations[a_i]));
    }

    // Trace the flow layer by layer to build the paths
    for (int t = 1; t <= current_makespan; ++t) {
        for (int a_i = 0; a_i < num_of_unassigned_agents; ++a_i) {
            Location last_loc = unassigned_agent_paths[a_i]->back().location;
            NodeIndex v = node_to_idx.at(FlowNode(FlowNode::Type::OUT, t - 1, last_loc));

            auto flow_it = flow.find(v);
            if (flow_it == flow.end()) return INFEASIBLE; // Flow broke down

            NodeIndex next_v = flow_it->second;
            FlowNode next_node = idx_to_node.at(next_v);

            if (next_node.type == FlowNode::Type::IN) {
                // The agent stood still (Wait track)
                unassigned_agent_paths[a_i]->push_back(State(t, next_node.location1));
            } else if (next_node.type == FlowNode::Type::EDGE_IN) {
                // The agent moved to a new tile. Trace through the EDGE gadget safely.
                auto structural_it1 = flow.find(next_v);
                if (structural_it1 == flow.end()) return INFEASIBLE;

                auto structural_it2 = flow.find(structural_it1->second);
                if (structural_it2 == flow.end()) return INFEASIBLE;

                next_v = structural_it2->second;
                next_node = idx_to_node.at(next_v);
                assert(next_node.type == FlowNode::Type::IN);

                unassigned_agent_paths[a_i]->push_back(State(t, next_node.location1));
            } else {
                throw std::runtime_error("Invalid next node type in UnassignedAgentsPlanner.");
            }
        }
    }
    return SUCCESS;
}

void UnassignedAgentsPlanner::print_flow() {
    std::cout << "Minimum cost_type flow: " << optimal_cost << std::endl;
    for (int i = 0; i < num_of_unassigned_agents; ++i) {
        std::cout << "---- Agent " << i << " ----" << std::endl;
        std::cout << "Cost     Flow" << std::endl;
        FlowNode curr_node = FlowNode(FlowNode::Type::OUT, 0, unassigned_agent_start_locations[i]);
        NodeIndex curr_vertex = node_to_idx.at(curr_node);
        NodeIndex next_vertex, edge_out, v_in;
        while (curr_node.time != current_makespan) {
            int cost = 0;

            next_vertex = flow.at(curr_vertex);
            cost += arc_cost.at(get_arc_idx(curr_vertex, next_vertex));
            FlowNode next_node = idx_to_node.at(next_vertex);

            if (next_node.type == FlowNode::Type::IN)  {
                std::cout << "[" << cost << "]  " << curr_node << " -> " << next_node << std::endl;
                curr_vertex = flow.at(next_vertex);
                curr_node = idx_to_node.at(curr_vertex);
            } else {
                edge_out = flow.at(next_vertex);
                cost += arc_cost.at(get_arc_idx(next_vertex, edge_out));
                v_in = flow.at(edge_out);
                cost += arc_cost.at(get_arc_idx(edge_out, v_in));
                std::cout << "[" << cost << "]  " << curr_node << " -> E" << next_node.location1 << "-" << next_node.location2
                          << " -> " << idx_to_node.at(v_in) << std::endl;
                curr_vertex = flow.at(v_in);
                curr_node = idx_to_node.at(curr_vertex);
            }
        }
        std::cout << "[*]  " << curr_node << " -> SINK * DONE" << std::endl << std::endl;
    }
}
