#ifndef MAPFUA_ASTAR_NODE_HPP
#define MAPFUA_ASTAR_NODE_HPP

#include "../common.hpp"

class AStarNode {
public:
    const State state;
    const int g_value;
    const int f_value;
    int q_value;
    shared_ptr<AStarNode> parent;

    AStarNode(State& state, int g_value, int f_value, int q_value = 0);
    AStarNode(State& state, int g_value, int f_value, shared_ptr<AStarNode>& parent, int q_value = 0);

    shared_ptr<TimedPath> get_path() const;
};

struct AStarNodeCompare{
    bool operator()(const shared_ptr<AStarNode>& lhs, const shared_ptr<AStarNode>& rhs) const {
//        return std::tie(lhs->f_value, lhs->q_value) > std::tie(rhs->f_value, rhs->q_value);
        return lhs->f_value > rhs->f_value;
    }
};

#endif //MAPFUA_ASTAR_NODE_HPP
