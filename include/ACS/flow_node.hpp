#ifndef FLOW_NODE_HPP
#define FLOW_NODE_HPP

#include "../common.hpp"

struct FlowNode {
    enum Type{
        OUT = 0,
        IN = 1,
        EDGE_IN = 2,
        EDGE_OUT = 3,
        SOURCE = 4,
        SINK = 5
    };

    Type type;
    int time;
    Location location1;
    Location location2;

    FlowNode(Type type, int time, Location location1, Location location2);
    FlowNode(Type type, int time, Location location);
    FlowNode(Type type, int time);

    bool operator==(const FlowNode& other) const;
    friend size_t hash_value(FlowNode const& n);
    friend std::ostream& operator<<(std::ostream& os, const FlowNode& node);
};

#endif //FLOW_NODE_HPP
