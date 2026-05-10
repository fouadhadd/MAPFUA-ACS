#include "constraints.hpp"


Constraints::Constraints(bool maintain_landmarks) : negative_vertex_constraints(), negative_edge_constraints(),
        positive_vertex_constraints(), positive_edge_constraints(), last_time_negative_vertex_constraint(),
        last_time_positive_vertex_constraint(-1), last_time_positive_edge_constraint(-1), maintain_landmarks(maintain_landmarks),
        v_landmarks(), e_landmarks() {}

Constraints::Constraints(const Constraints &constraints) = default;

void Constraints::add_negative_vertex_constraint(VertexConstraint &v_constraint) {
    assert(positive_vertex_constraints.find(v_constraint.time) == positive_vertex_constraints.end() ||
           positive_vertex_constraints.at(v_constraint.time) != v_constraint.location);
    auto it = negative_vertex_constraints.find(v_constraint.time);
    if (it == negative_vertex_constraints.end()) {
        negative_vertex_constraints[v_constraint.time] = unordered_set<Location>();
        negative_vertex_constraints.at(v_constraint.time).insert(v_constraint.location);
    } else{
        it->second.insert(v_constraint.location);
    }
    auto it2 = last_time_negative_vertex_constraint.find(v_constraint.location);
    if (it2 == last_time_negative_vertex_constraint.end() || it2->second < v_constraint.time) {
        last_time_negative_vertex_constraint[v_constraint.location] = v_constraint.time;
    }
}

void Constraints::add_negative_edge_constraint(EdgeConstraint &e_constraint) {
    auto it = negative_edge_constraints.find(e_constraint.time);
    if (it == negative_edge_constraints.end()) {
        negative_edge_constraints.insert({e_constraint.time, unordered_set<std::pair<Location, Location>>()});
        negative_edge_constraints.at(e_constraint.time).insert(std::make_pair(e_constraint.from, e_constraint.to));
    } else {
        auto tmp = std::make_pair(e_constraint.from, e_constraint.to);
        it->second.insert(tmp);
    }
}

void Constraints::add_positive_vertex_constraint(VertexConstraint &v_constraint) {
    assert(positive_vertex_constraints.find(v_constraint.time) == positive_vertex_constraints.end());
    positive_vertex_constraints.insert({v_constraint.time, v_constraint.location});

    if (maintain_landmarks) {
        v_landmarks.emplace(v_constraint.time, v_constraint.location);
    }

    if (last_time_positive_vertex_constraint < v_constraint.time) {
        last_time_positive_vertex_constraint = v_constraint.time;
    }
}

void Constraints::add_positive_edge_constraint(EdgeConstraint &e_constraint) {
    assert(positive_edge_constraints.find(e_constraint.time) == positive_edge_constraints.end());
    assert(negative_vertex_constraints.find(e_constraint.time) == negative_vertex_constraints.end() ||
           negative_vertex_constraints.at(e_constraint.time).find(e_constraint.from) ==
           negative_vertex_constraints.at(e_constraint.time).end());
    assert(negative_vertex_constraints.find(e_constraint.time + 1) == negative_vertex_constraints.end() ||
             negative_vertex_constraints.at(e_constraint.time + 1).find(e_constraint.to) ==
             negative_vertex_constraints.at(e_constraint.time + 1).end());
    positive_edge_constraints[e_constraint.time] = std::make_pair(e_constraint.from, e_constraint.to);

    if (maintain_landmarks){
        e_landmarks.emplace(e_constraint.time, std::make_pair(e_constraint.from, e_constraint.to));
    }

    if (last_time_positive_edge_constraint < e_constraint.time) {
        last_time_positive_edge_constraint = e_constraint.time;
    }
}

bool Constraints::is_state_valid(const State &state) const {
    auto it = negative_vertex_constraints.find(state.time);
    if (it != negative_vertex_constraints.end() && it->second.find(state.location) != it->second.end()) {
        return false;
    }
    auto it2 = positive_vertex_constraints.find(state.time);
    if (it2 != positive_vertex_constraints.end() && it2->second != state.location) {
        return false;
    }
    return true;
}

bool Constraints::is_transition_valid(const State &from_s, const State &to_s) const {
    assert(from_s.time + 1 == to_s.time);
    auto it = negative_edge_constraints.find(from_s.time);
    std::pair<Location, Location> s_pair = std::make_pair(from_s.location, to_s.location);
    if (it != negative_edge_constraints.end() && it->second.find(s_pair) != it->second.end()) {
        return false;
    }
    auto it2 = positive_edge_constraints.find(from_s.time);
    if (it2 != positive_edge_constraints.end() && it2->second != s_pair) {
        return false;
    }
    return true;
}

bool Constraints::can_stay(const State &state) const {
    // assumes that the state is valid
    if (last_time_positive_vertex_constraint > state.time) {
        return false;
    } else if (last_time_positive_vertex_constraint == state.time) {
        auto it = positive_vertex_constraints.find(state.time);
        assert(it != positive_vertex_constraints.end());
        if (it->second != state.location) {
            return false;
        }
    }

    if (last_time_positive_edge_constraint >= state.time) {
        return false;
    }

    auto it2 = last_time_negative_vertex_constraint.find(state.location);
    if (it2 != last_time_negative_vertex_constraint.end() && it2->second >= state.time) {
        return false;
    }

    return true;
}

unordered_map<int, std::pair<Location, Location>> Constraints::get_positive_edge_constraints() const {
    return positive_edge_constraints;
}
