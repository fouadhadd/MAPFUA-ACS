#ifndef MAPFUA_ACS_NODE_HPP
#define MAPFUA_ACS_NODE_HPP

#include "../common.hpp"
#include "../constraints/constraints.hpp"

class ACSNode {
public:
    int idx;
    vector<shared_ptr<TimedPath>> assigned_paths;
    int cost;
    vector<shared_ptr<Constraints>> constraint_table;

    int iteration;
    const shared_ptr<ACSNode> parent;

    ACSNode(int idx, vector<shared_ptr<TimedPath>>& obs_paths, int cost,
               vector<shared_ptr<Constraints>> &constraint_table,
               const shared_ptr<ACSNode>& parent = nullptr);
    explicit ACSNode(int idx, const shared_ptr<ACSNode>& node);

    shared_ptr<Conflict> get_first_assigned_conflict() const;
};

struct ACSNodeCompare{
    bool operator()(const shared_ptr<ACSNode>& lhs, const shared_ptr<ACSNode>& rhs) const {
//        return lhs->cost > rhs->cost;
        return std::tie(lhs->cost, lhs->idx) > std::tie(rhs->cost, rhs->idx);
    }
};

#endif //MAPFUA_ACS_NODE_HPP
