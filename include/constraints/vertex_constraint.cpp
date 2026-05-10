#include "vertex_constraint.hpp"

VertexConstraint::VertexConstraint(int time, Location location) : time(time), location(location) {}

bool VertexConstraint::operator<(const VertexConstraint& other) const {
    return std::tie(time, location) < std::tie(other.time, other.location);
}

bool VertexConstraint::operator==(const VertexConstraint& other) const {
    return std::tie(time, location) == std::tie(other.time, other.location);
}

std::ostream& operator<<(std::ostream& os, const VertexConstraint& c){
    return os << "VC(" << c.time << ", " << c.location << ")";
}

size_t hash_value(VertexConstraint const& vc){
    size_t seed = 0;
    boost::hash_combine(seed, vc.time);
    boost::hash_combine(seed, vc.location);
    return seed;
}

