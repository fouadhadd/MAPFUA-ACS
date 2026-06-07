#include "../ACS/flow_node.hpp"


FlowNode::FlowNode(Type type, int time, Location location1, Location location2) : type(type), time(time), location1(location1), location2(location2) {}

FlowNode::FlowNode(Type type, int time, Location location) : FlowNode(type, time, location, Location(-1 , -1)) {}

FlowNode::FlowNode(Type type, int time) : FlowNode(type, time, Location(-1, -1)) {}


bool FlowNode::operator==(const FlowNode &other) const {
    return location1 == other.location1 && location2 == other.location2 && type == other.type && time == other.time;
}

size_t hash_value(FlowNode const& n){
    size_t seed = 0;
    boost::hash_combine(seed, n.location1);
    boost::hash_combine(seed, n.location2);
    boost::hash_combine(seed, n.type);
    boost::hash_combine(seed, n.time);
    return seed;
}

std::ostream& operator<<(std::ostream& os, const FlowNode& node){
    if (node.type == FlowNode::Type::OUT)
        os << "(" << node.location1 << ", OUT, " << node.time << ")";
    else if (node.type == FlowNode::Type::IN)
        os << "(" << node.location1 << ", IN, " << node.time << ")";
    else if (node.type == FlowNode::Type::EDGE_IN)
        os << "(" << node.location1 << ", " << node.location2 << ", EDGE_IN, " << node.time << ")";
    else if (node.type == FlowNode::Type::EDGE_OUT)
        os << "(" << node.location1 << ", " << node.location2 << ", EDGE_OUT, " << node.time << ")";
    else if (node.type == FlowNode::Type::SINK)
        os << "(" << node.location1 << ", SINK, " << node.time << ")";
    else if (node.type == FlowNode::Type::SOURCE)
        os << "(" << node.location1 << ", SOURCE, " << node.time << ")";
    else
        assert(false);
    return os;
}

