#include "acs.hpp"

ACS::ACS(int map_rows, int map_cols, vector<vector<CellType>> &map,
         vector<Location> &agent_start_locations, vector<Location> &assigned_goal_locations,
         vector<Location> &unassigned_start_locations, bool verbose) :
        num_of_assigned(static_cast<int>(agent_start_locations.size())),
        num_of_unassigned(static_cast<int>(unassigned_start_locations.size())),
        verbose(verbose),
        unassigned_planner(map_rows, map_cols, map, unassigned_start_locations, verbose) {

        for (int i = 0; i < num_of_assigned; i++) {
            assigned_start_ts.push_back(0);
            State s_state(0, agent_start_locations[i]);

            // Initializing a low-level solver for each assigned agent
            solvers.emplace_back(map_rows, map_cols, map, s_state, assigned_goal_locations[i]);
    }
}

ACS::ACS(Scenario &scenario, bool verbose) :
    ACS(scenario.map_rows, scenario.map_cols, scenario.map, scenario.assigned_start_locations,
        scenario.assigned_goal_locations, scenario.unassigned_start_locations, verbose) {}

shared_ptr<Plan> ACS::solve(int timeout) {
    bool time_limit_exceeded;
    return solve(timeout, time_limit_exceeded);
}

shared_ptr<Plan> ACS::solve(int time_limit, bool& time_limit_exceeded) {
    time_limit_exceeded = false;
    Timer solve_timer;
    solve_timer.reset();
    OpenList open_list;

    open_list.push(get_root_node());
    iterations = 0;

    while (!open_list.empty()) {
        solve_timer.stop();
        if (solve_timer.elapsed_seconds() > time_limit) {
            time_limit_exceeded = true;
            return nullptr;
        }

        shared_ptr<ACSNode> node = open_list.top();
        node->iteration = ++iterations;
        open_list.pop();

        if (verbose){
            std::cout << "Iteration: " << iterations << " Cost: " << node->cost << std::endl;
        }

        shared_ptr<Conflict> conflict = node->get_first_assigned_conflict();

        if (conflict) {
            if (verbose) {
                std::cout << "Conflict detected: " << *conflict << std::endl;
            }
            auto children = resolve_assigned_conflict(node, conflict);
            if (children.first) open_list.push(children.first);
            if (children.second) open_list.push(children.second);
            continue;
        }

        if (verbose) {
            std::cout << "Conflict-free backbone found. Starting Phase 2: Planning unassigned agents..." << std::endl;
        }

        // Phase 2: Plan unassigned agents using network flow
        if (num_of_unassigned > 0) {
            auto unassigned_result = unassigned_planner.find_paths(node->assigned_paths, node->cost);

            if (unassigned_result == UnassignedAgentsPlanner::CONFLICT) {
                if (verbose) {
                    std::cout << "Phase 2 Conflict: " << *unassigned_planner.capacity_conflict << std::endl;
                }

                // Branch the tree to break the trap
                auto children = resolve_capacity_conflict(node, unassigned_planner.capacity_conflict);
                if (children.first) open_list.push(children.first);
                if (children.second) open_list.push(children.second);

                continue;
            }
            else if (unassigned_result == UnassignedAgentsPlanner::INFEASIBLE) {
                if (verbose) std::cout << "Phase 2 failed. Infeasible. Backtracking..." << std::endl;
                continue;
            }

            if (verbose) {
                std::cout << "Phase 2 succeeded: Unassigned agents planned successfully." << std::endl;
            }
        }

        return std::make_shared<Plan>(node->assigned_paths, unassigned_planner.unassigned_agent_paths);
    }
    return nullptr;
}

shared_ptr<ACSNode> ACS::get_root_node() {
    vector<shared_ptr<Constraints>> constraint_table(num_of_assigned, std::make_shared<Constraints>(true));
    vector<shared_ptr<TimedPath>> assigned_paths(num_of_assigned, nullptr);
    int cost = 0;

    for (int i = 0; i < num_of_assigned; i++) {
        assigned_paths[i] = plan_single_assigned_path(i, constraint_table[i]);
        if (assigned_paths[i] == nullptr) continue;

        cost = std::max(cost, assigned_paths[i]->back().time);
    }
    return std::make_shared<ACSNode>(node_count++, assigned_paths, cost, constraint_table);
}

shared_ptr<TimedPath> ACS::plan_single_assigned_path(int agent_id, shared_ptr<Constraints> constraints) {
    shared_ptr<TimedPath> path = solvers[agent_id].find_path_use_landmarks(constraints);
    if (path != nullptr)
        path->erase_from_beginning();
    return path;
}

std::pair<shared_ptr<ACSNode>, shared_ptr<ACSNode>>
ACS::resolve_assigned_conflict(const shared_ptr<ACSNode>& node, const shared_ptr<Conflict>& conflict) {
    VertexConstraint v_cons(conflict->time, conflict->loc1);
    EdgeConstraint e_cons(conflict->time, conflict->loc1, conflict->loc2);
    EdgeConstraint e_cons_reversed(conflict->time, conflict->loc2, conflict->loc1);

    // Negative Constraint Branch
    shared_ptr<ACSNode> neg_child = nullptr;
    if (conflict->time >= assigned_start_ts[conflict->agent1]) {
        shared_ptr<Constraints> neg_cons = std::make_shared<Constraints>(*node->constraint_table[conflict->agent1]);
        if (conflict->type == Conflict::Type::Vertex)
            neg_cons->add_negative_vertex_constraint(v_cons);
        else
            neg_cons->add_negative_edge_constraint(e_cons);

        shared_ptr<ACSNode> tmp_neg_child = std::make_shared<ACSNode>(node_count++, node);
        tmp_neg_child->constraint_table[conflict->agent1] = neg_cons;

        shared_ptr<TimedPath> neg_path = plan_single_assigned_path(conflict->agent1, neg_cons);
        if (neg_path) {
            neg_child = tmp_neg_child;
            neg_child->cost = std::max(neg_child->cost, neg_path->back().time);
            neg_child->assigned_paths[conflict->agent1] = neg_path;
        }
    }

    if (conflict->time < assigned_start_ts[conflict->agent2]) {
        return {neg_child, nullptr};
    }

    // Positive Constraint Branch
    shared_ptr<ACSNode> pos_child = std::make_shared<ACSNode>(node_count++, node);
    for (int i = 0; i < num_of_assigned; i++) {
        shared_ptr<Constraints> pos_cons = std::make_shared<Constraints>(*node->constraint_table[i]);
        pos_child->constraint_table[i] = pos_cons;

        if (conflict->type == Conflict::Type::Vertex){
            if (i == conflict->agent1) {
                pos_cons->add_positive_vertex_constraint(v_cons);
                continue;
            }
            pos_cons->add_negative_vertex_constraint(v_cons);
        } else {
            if (i == conflict->agent1) {
                pos_cons->add_positive_edge_constraint(e_cons);
                continue;
            }
            pos_cons->add_negative_edge_constraint(e_cons_reversed);
        }

        // Re-planning any agent forced to move by the positive constraint
        shared_ptr<TimedPath> pos_path = plan_single_assigned_path(i, pos_cons);
        if (!pos_path){
            pos_child = nullptr;
            break;
        }
        pos_child->cost = std::max(pos_child->cost, pos_path->back().time);
        pos_child->assigned_paths[i] = pos_path;
    }
    return {neg_child, pos_child};
}

std::pair<shared_ptr<ACSNode>, shared_ptr<ACSNode>>
ACS::resolve_capacity_conflict(const shared_ptr<ACSNode> &node, const shared_ptr<CapacityConflict> &conflict) {
    bool is_vertex = (conflict->location1 == conflict->location2);
    VertexConstraint v_cons(conflict->time, conflict->location1);
    EdgeConstraint e_cons(conflict->time, conflict->location1, conflict->location2);

    // Only Negative Constraint Branch
    shared_ptr<ACSNode> neg_child = std::make_shared<ACSNode>(node_count++, node);
    shared_ptr<Constraints> neg_cons = std::make_shared<Constraints>(*node->constraint_table[conflict->assigned_id]);

    if (is_vertex) {
        neg_cons->add_negative_vertex_constraint(v_cons);
    } else {
        neg_cons->add_negative_edge_constraint(e_cons);
    }

    neg_child->constraint_table[conflict->assigned_id] = neg_cons;
    shared_ptr<TimedPath> neg_path = plan_single_assigned_path(conflict->assigned_id, neg_cons);

    if (neg_path) {
        neg_child->cost = std::max(neg_child->cost, neg_path->back().time);
        neg_child->assigned_paths[conflict->assigned_id] = neg_path;

        // Return ONLY the negative branch. The positive branch is mathematically invalid here.
        return {neg_child, nullptr};
    }

    return {nullptr, nullptr};
}