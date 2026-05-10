#ifndef MAPFUA_CONSTRAINTS_HPP
#define MAPFUA_CONSTRAINTS_HPP

#include "../common.hpp"
#include "vertex_constraint.hpp"
#include "edge_constraint.hpp"



class Constraints {
public:
    unordered_map<int, unordered_set<Location>> negative_vertex_constraints;
    unordered_map<int, unordered_set<std::pair<Location, Location>>> negative_edge_constraints;
    unordered_map<int, Location> positive_vertex_constraints;
    unordered_map<int, std::pair<Location, Location>> positive_edge_constraints;
    unordered_map<Location, int> last_time_negative_vertex_constraint;
    int last_time_positive_vertex_constraint;
    int last_time_positive_edge_constraint;

    bool maintain_landmarks;
    map<int, Location> v_landmarks;
    map<int, std::pair<Location, Location>> e_landmarks;

    explicit Constraints(bool maintain_landmarks = false);
    Constraints(const Constraints &constraints);

    void add_negative_vertex_constraint(VertexConstraint &v_constraint);
    void add_negative_edge_constraint(EdgeConstraint &e_constraint);
    void add_positive_vertex_constraint(VertexConstraint &v_constraint);
    void add_positive_edge_constraint(EdgeConstraint &e_constraint);

    bool is_state_valid(const State &state) const;
    bool is_transition_valid(const State &from_s, const State &to_s) const;
    bool can_stay(const State &state) const;

    unordered_map<int, std::pair<Location, Location>> get_positive_edge_constraints() const;
};


#endif //MAPFUA_CONSTRAINTS_HPP
