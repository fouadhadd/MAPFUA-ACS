#ifndef MAPFUA_ASTAR_HPP
#define MAPFUA_ASTAR_HPP

#include "astar_node.hpp"
#include "../constraints/constraints.hpp"

class AStar {
private:
    vector<vector<CellType>> map;
    int map_rows;
    int map_cols;
    Location start_location;
    int start_time;
    Location goal_location;
    int goal_time;
    const int original_start_time, original_goal_time;
    const Location original_start_location, original_goal_location;

    constexpr static const int dx[5] = {0, 0, 1, 0, -1};
    constexpr static const int dy[5] = {0, 1, 0, -1, 0};

    typedef boost::heap::d_ary_heap<shared_ptr<AStarNode>, boost::heap::arity<2>, boost::heap::mutable_<true>,
            boost::heap::compare<AStarNodeCompare>> OpenList;

    typedef OpenList::handle_type NodeHandle;

public:
    int total_moves = 0;

    AStar(int map_rows, int map_cols, vector<vector<CellType>> &map, State &start_state, Location &goal_location, int goal_time);
    AStar(int map_rows, int map_cols, vector<vector<CellType>> &map, State &start_state, Location &goal_location);

    shared_ptr<TimedPath> find_path(shared_ptr<Constraints> &constraints);

    shared_ptr<TimedPath> find_path_use_landmarks(shared_ptr<Constraints> &constraints);

    void reset();

private:
    int calculate_h(const State &curr_state) const;

    bool is_goal_state(const State &state);
};


#endif //MAPFUA_ASTAR_HPP
