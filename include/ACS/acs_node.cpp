#include "acs_node.hpp"


ACSNode::ACSNode(int idx, vector<shared_ptr<TimedPath>> &assigned_paths, int cost,
    vector<shared_ptr<Constraints>> &constraint_table, const shared_ptr<ACSNode>& parent):
        idx(idx), assigned_paths(assigned_paths), cost(cost), constraint_table(constraint_table),
        iteration(-1), parent(parent) {}

ACSNode::ACSNode(int idx, const shared_ptr<ACSNode> &node) :
        ACSNode(idx, node->assigned_paths, node->cost, node->constraint_table) {}

shared_ptr<Conflict> ACSNode::get_first_assigned_conflict() const {
    int last_t = 0;
    int first_t = std::numeric_limits<int>::max();
    for (const auto &assigned_path : assigned_paths) {
        last_t = std::max(last_t, assigned_path->back().time);
        first_t = std::min(first_t, assigned_path->front().time);
    }
    auto num_of_obs = assigned_paths.size();

    for (int t = first_t; t <= last_t; t++) {
        for (size_t i = 0; i < num_of_obs; i++) {
            const Location &loc_i = assigned_paths[i]->get_location_at_time(t);
            for (size_t j = i + 1; j < num_of_obs; j++) {
                if ((t < assigned_paths[i]->front().time && t < assigned_paths[j]->front().time) ||
                    (t >= assigned_paths[i]->back().time && t >= assigned_paths[j]->back().time)) {
                    continue;
                }

                //  check vertex conflict
                const Location &loc_j = assigned_paths[j]->get_location_at_time(t);

                if (loc_i == loc_j) {
                    return std::make_shared<Conflict>(t, i, j, loc_i);
                }

                if (t == last_t || t < assigned_paths[i]->front().time || t < assigned_paths[j]->front().time ||
                    t >= assigned_paths[i]->back().time || t >= assigned_paths[j]->back().time) {
                    continue;
                }

                // check edge conflict
                const Location &loc_i_next = assigned_paths[i]->get_location_at_time(t + 1);
                const Location &loc_j_next = assigned_paths[j]->get_location_at_time(t + 1);
                if (loc_i == loc_j_next && loc_j == loc_i_next) {
                    return std::make_shared<Conflict>(t, i, j, loc_i, loc_i_next);
                }
            }
        }
    }

    return nullptr;
}
