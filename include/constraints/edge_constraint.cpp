#include "edge_constraint.hpp"


EdgeConstraint::EdgeConstraint(int time, Location from, Location to) : time(time), from(from), to(to) {}

bool EdgeConstraint::operator<(const EdgeConstraint& other) const {
    return std::tie(time, from, to) < std::tie(other.time, other.from, other.to);
}

bool EdgeConstraint::operator==(const EdgeConstraint& other) const {
    return std::tie(time, from, to) == std::tie(other.time, other.from, other.to);
}

std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c){
    return os << "EC(" << c.time << ", " << c.from << "->" << c.to << ")";
}

size_t hash_value(EdgeConstraint const& ec){
    size_t seed = 0;
    boost::hash_combine(seed, ec.time);
    boost::hash_combine(seed, ec.from);
    boost::hash_combine(seed, ec.to);
    return seed;
}
