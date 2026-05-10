#include "common.hpp"
#include <algorithm>

int Plan::get_makespan() {
    if (makespan != 0) {
        return makespan;
    }
    for (const auto &path : assigned_paths) {
        if (!path->empty()) {
            makespan = std::max(makespan, path->back().time);
        }
    }
    for (const auto &path : unassigned_paths) {
        if (!path->empty()) {
            makespan = std::max(makespan, path->back().time);
        }
    }
    return makespan;
}

bool Plan::validate_plan(int map_rows, int map_cols, const vector<vector<CellType>> &map,
                         const vector<Location> &assigned_start_locations,
                         const vector<Location> &assigned_goal_locations,
                         const vector<Location> &unassigned_start_locations) {
    if (assigned_paths.size() != assigned_start_locations.size()) return false;
    if (unassigned_paths.size() != unassigned_start_locations.size()) return false;

    int total_makespan = get_makespan();

    // Combine all paths for collision checking
    vector<shared_ptr<TimedPath>> all_paths = assigned_paths;
    all_paths.insert(all_paths.end(), unassigned_paths.begin(), unassigned_paths.end());

    for (size_t i = 0; i < assigned_paths.size(); ++i) {
        const auto &path = assigned_paths.at(i);
        if (path->empty()) return false;
        if (path->get_location_at_time(0) != assigned_start_locations.at(i)) return false;
        if (path->back().location != assigned_goal_locations.at(i)) return false;
    }

    for (size_t i = 0; i < unassigned_paths.size(); ++i) {
        const auto &path = unassigned_paths.at(i);
        if (path->empty()) return false;
        if (path->get_location_at_time(0) != unassigned_start_locations.at(i)) return false;
    }

    for (int t = 1; t <= total_makespan; ++t) {
        for (size_t i = 0; i < all_paths.size(); ++i) {
            Location loc_i = all_paths[i]->get_location_at_time(t);
            Location prev_loc_i = all_paths[i]->get_location_at_time(t - 1);

            // Map constraints
            if (loc_i.x < 0 || loc_i.x >= map_rows || loc_i.y < 0 || loc_i.y >= map_cols || map[loc_i.x][loc_i.y] == CellType::BLOCKED) {
                return false;
            }

            for (size_t j = i + 1; j < all_paths.size(); ++j) {
                Location loc_j = all_paths[j]->get_location_at_time(t);
                Location prev_loc_j = all_paths[j]->get_location_at_time(t - 1);

                // Vertex conflict
                if (loc_i == loc_j) return false;

                // Edge conflict
                if (loc_i == prev_loc_j && loc_j == prev_loc_i) return false;
            }
        }
    }

    return true;
}
