#include "astar.hpp"


AStar::AStar(int map_rows, int map_cols, vector<vector<CellType>> &map, State &start_state, Location &goal_location, int goal_time) :
        map(map), map_rows(map_rows), map_cols(map_cols), start_location(start_state.location),
        start_time(start_state.time), goal_location(goal_location), goal_time(goal_time),
        original_start_time(start_time), original_goal_time(goal_time),
        original_start_location(start_location), original_goal_location(goal_location) {}

AStar::AStar(int map_rows, int map_cols, vector<vector<CellType>> &map, State &start_state, Location &goal_location) :
        AStar(map_rows, map_cols, map, start_state, goal_location, -1){}

shared_ptr<TimedPath> AStar::find_path(shared_ptr<Constraints> &constraints) {
    total_moves = 0;
    State start_state(start_time, start_location);
    if (!constraints->is_state_valid(start_state)) {
        return nullptr;
    }
    int next_g = 0;
    int next_q = 0;
    shared_ptr<AStarNode> start_node(new AStarNode(start_state, next_g, calculate_h(start_state)));
    OpenList open_list;
    unordered_map<State, NodeHandle> state_to_handle;
    state_to_handle.emplace(start_state, open_list.push(start_node));
    unordered_set<State> closed_list;

    while (!open_list.empty()) {
        shared_ptr<AStarNode> current_node = open_list.top();

        if (is_goal_state(current_node->state) &&
                (goal_time != -1 || constraints->can_stay(current_node->state))) {
            total_moves = current_node->q_value;
            return current_node->get_path();
        }
        closed_list.insert(current_node->state);
        open_list.pop();
        state_to_handle.erase(current_node->state);
        next_g = current_node->g_value + 1;

        if (goal_time != -1 && current_node->state.time + 1 > goal_time)
            continue;

        for (int i=0; i< 5; i++){
            Location next_location(current_node->state.location.x + dx[i], current_node->state.location.y + dy[i]);
            if (i > 0 && (next_location.x < 0 || next_location.x >= map_rows || next_location.y < 0 || next_location.y >= map_cols ||
                    map[next_location.x][next_location.y] == CellType::BLOCKED)) {
                continue;
            }

            State next_state(current_node->state.time + 1, next_location);
            next_q = current_node->q_value + (i > 0);

            if (constraints->is_state_valid(next_state) &&
                    constraints->is_transition_valid(current_node->state, next_state) &&
                    closed_list.find(next_state) == closed_list.end()) {
                auto sth_iter = state_to_handle.find(next_state);
                if (sth_iter != state_to_handle.end()){
                    auto &in_state = *sth_iter->second;
                    if (next_q < in_state->q_value) {
                        in_state->q_value = next_q;
                        in_state->parent = current_node;
                        open_list.update(sth_iter->second);
                    }
                    continue;
                }
                shared_ptr<AStarNode> next_node(new AStarNode(next_state, next_g, next_g + calculate_h(next_state), current_node, next_q));
                state_to_handle.emplace(next_state, open_list.push(next_node));
            }
        }
    }

    return nullptr;
}

int AStar::calculate_h(const State &curr_state) const {
    int loc_dist = abs(curr_state.location.x - goal_location.x) + abs(curr_state.location.y - goal_location.y);
    if (goal_time == -1) {
        return loc_dist;
    }
    return std::max(loc_dist, goal_time - curr_state.time);
}

bool AStar::is_goal_state(const State &state) {
    return state.location == goal_location && (goal_time == -1 || state.time == goal_time);
}

shared_ptr<TimedPath> AStar::find_path_use_landmarks(shared_ptr<Constraints> &constraints) {
    int real_total_moves = 0;
    auto v_landmarks_it = constraints->v_landmarks.begin();

    while (v_landmarks_it != constraints->v_landmarks.end() && v_landmarks_it->first <= start_time) {
        assert(v_landmarks_it->second == start_location);
        v_landmarks_it++;
    }

    auto e_landmarks_it = constraints->e_landmarks.begin();
    shared_ptr<TimedPath> path = std::make_shared<TimedPath>();
    path->push_back(State(start_time, start_location));

    bool vertex_landmark;
    while (v_landmarks_it != constraints->v_landmarks.end() || e_landmarks_it != constraints->e_landmarks.end()) {
        vertex_landmark = false;
        if (v_landmarks_it != constraints->v_landmarks.end() && e_landmarks_it != constraints->e_landmarks.end()) {
            if (v_landmarks_it->first <= e_landmarks_it->first) {
                vertex_landmark = true;
            }
        } else if (v_landmarks_it != constraints->v_landmarks.end()) {
            vertex_landmark = true;
        }

        if (vertex_landmark) {
            goal_time = v_landmarks_it->first;
            goal_location = v_landmarks_it->second;
        } else {
            goal_time = e_landmarks_it->first;
            goal_location = e_landmarks_it->second.first;
        }

        assert(goal_time >= start_time);

        shared_ptr<TimedPath> seg_path = find_path(constraints);
        if (!seg_path) {
            reset();
            return nullptr;
        }

        real_total_moves += total_moves;
        path->insert(path->end(), seg_path->begin() + 1, seg_path->end());

        if (vertex_landmark) {
            start_time = goal_time;
            start_location = goal_location;
            v_landmarks_it++;
        } else {
            State next_state(goal_time + 1, e_landmarks_it->second.second);
            if (!constraints->is_state_valid(next_state) || !constraints->is_transition_valid(path->back(), next_state)) {
                reset();
                return nullptr;
            }
            path->push_back(next_state);
            real_total_moves++;
            start_time = goal_time + 1;
            start_location = e_landmarks_it->second.second;
            e_landmarks_it++;
        }
    }

    goal_time = original_goal_time;
    goal_location = original_goal_location;
    shared_ptr<TimedPath> last_seg_path = find_path(constraints);
    start_time = original_start_time;
    start_location = original_start_location;

    if (!last_seg_path) {
        return nullptr;
    }
    total_moves += real_total_moves;
    path->insert(path->end(), last_seg_path->begin() + 1, last_seg_path->end());
    return path;
}

void AStar::reset() {
    start_time = original_start_time;
    start_location = original_start_location;
    goal_time = original_goal_time;
    goal_location = original_goal_location;
    total_moves = 0;
}
