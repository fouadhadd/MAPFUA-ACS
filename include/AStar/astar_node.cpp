#include "astar_node.hpp"

AStarNode::AStarNode(State& state, int g_value, int f_value, int q_value) :
        state(state), g_value(g_value), f_value(f_value), q_value(q_value), parent(nullptr) {}

AStarNode::AStarNode(State& state, int g_value, int f_value, shared_ptr<AStarNode> &parent, int q_value) :
        state(state), g_value(g_value), f_value(f_value), q_value(q_value), parent(parent) {}

shared_ptr<TimedPath> AStarNode::get_path() const{
    shared_ptr<TimedPath> path = std::make_shared<TimedPath>();
    path->push_back(this->state);
    shared_ptr<AStarNode> current = this->parent;
    while (current) {
        path->push_back(current->state);
        current = current->parent;
    }
    std::reverse(path->begin(), path->end());
    return path;
}

