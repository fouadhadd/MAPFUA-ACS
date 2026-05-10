#ifndef MAPFUA_COMMON_HPP
#define MAPFUA_COMMON_HPP

#include <tuple>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cassert>
#include <chrono>
#include <set>
#include <map>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/heap/d_ary_heap.hpp>


using std::shared_ptr;
using std::string;
using std::vector;
using std::array;
using std::set;
using std::map;
using boost::unordered_set;
using boost::unordered_map;


enum CellType {
    PASSABLE = 0,
    BLOCKED = 1
};


struct Location {
    int x;
    int y;

    Location() : x(-1), y(-1) {}
    Location(int x, int y) : x(x), y(y) {}

    bool operator<(const Location &other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator==(const Location &other) const {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    bool operator!=(const Location &other) const {
        return not (*this == other);
    }

    friend std::ostream &operator<<(std::ostream &os, const Location &c) {
        return os << "(" << c.x << ", " << c.y << ")";
    }

    inline friend size_t hash_value(Location const& location){
        size_t seed = 0;
        boost::hash_combine(seed, location.x);
        boost::hash_combine(seed, location.y);
        return seed;
    }
};

inline int manhattan_distance(const Location &a, const Location &b){
    return abs(a.x - b.x) + abs(a.y - b.y);
}


struct State {
    int time;
    Location location;

    State() : time(-1), location() {}
    State(int time, Location location) : time(time), location(location) {}
    State(int time, int x, int y) : time(time), location(x, y) {}

    bool operator==(const State &s) const {
        return time == s.time && location == s.location;
    }

    bool operator!=(const State &s) const {
        return not (*this == s);
    }

    friend std::ostream &operator<<(std::ostream &os, const State &s) {
        return os << s.time << ": " << s.location;
    }

    inline friend size_t hash_value(State const& state){
        size_t seed = 0;
        boost::hash_combine(seed, state.time);
        boost::hash_combine(seed, state.location);
        return seed;
    }
};



class TimedPath : public vector<State> {
public:
    using vector<State>::vector;

    inline Location get_location_at_time(int time) const{
        if (time < 0) throw std::out_of_range("Time cannot be negative");
        int s_time = this->front().time;
        if (time <= s_time) return this->front().location;
        if (time >= this->back().time) return this->back().location;
        return this->at(time - s_time).location;
    }

    friend std::ostream &operator<<(std::ostream &os, const TimedPath &p) {
        os << "{";
        for (size_t i = 0; i < p.size(); ++i) {
            os << p[i] << (i == p.size() - 1 ? "" : ", ");
        }
        return os << "}";
    }

    void erase_from_beginning() {
        if (this->empty()) return;
        auto iter = this->begin();
        while (++iter != this->end()) {
            if (iter->location != this->front().location) break;
        }
        if (iter != this->end() && --iter != this->begin())
            this->erase(this->begin(), iter);
    }


};

inline
string to_string_vec(const vector<shared_ptr<TimedPath>>& paths) {
    std::ostringstream os;
    os << "Paths: " << std::endl;
    for (size_t i = 0; i < paths.size(); ++i) {
        os << "Agent " << i << ": " << *paths.at(i) << std::endl;
    }
    os << std::endl;
    return os.str();
}


class Plan {
    int makespan{0};
public:
    vector<shared_ptr<TimedPath>> assigned_paths;
    vector<shared_ptr<TimedPath>> unassigned_paths;

    Plan(vector<shared_ptr<TimedPath>>& assigned_paths, vector<shared_ptr<TimedPath>>& unassigned_paths)
    : assigned_paths(assigned_paths), unassigned_paths(unassigned_paths) {}

    int get_makespan();

    friend std::ostream &operator<<(std::ostream &os, Plan &p) {
        os << "Plan:" << std::endl;
        for (size_t i = 0; i < p.assigned_paths.size(); ++i) {
            os << "Assigned Agent " << i << ": " << *p.assigned_paths.at(i) << std::endl;
        }
        for (size_t i = 0; i < p.unassigned_paths.size(); ++i) {
            os << "Unassigned Agent " << i << ": " << *p.unassigned_paths.at(i) << std::endl;
        }
        os << "Makespan: " << p.get_makespan() << std::endl;
        return os;
    }

    bool validate_plan(int map_rows, int map_cols, const vector<vector<CellType>>& map,
                       const vector<Location>& assigned_start_locations,
                       const vector<Location>& assigned_goal_locations,
                       const vector<Location>& unassigned_start_locations);
};

struct Conflict {
    enum Type {
        Vertex,
        Edge,
    };

    int time;
    int agent1;
    int agent2;
    Type type;

    Location loc1;
    Location loc2;

    Conflict(int time, int agent1, int agent2, Location loc1) :
            time(time), agent1(agent1), agent2(agent2), type(Vertex), loc1(loc1), loc2() {}
    Conflict(int time, int agent1, int agent2, Location loc1, Location loc2) :
            time(time), agent1(agent1), agent2(agent2), type(Edge), loc1(loc1), loc2(loc2) {}

    friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
        switch (c.type) {
            case Vertex:
                return os << "VertexConflict(t=" << c.time << ", a=" << c.agent1 << ", a2=" << c.agent2
                << ", v=" << c.loc1 << ")";
            case Edge:
                return os << "EdgeConflict(t=" << c.time << ", a1=" << c.agent1 << ", a2=" << c.agent2
                << ", e=" << c.loc1 << "->" << c.loc2 << ")";
        }
        return os;
    }
};



class Timer {
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;

public:
    Timer() : start(std::chrono::high_resolution_clock::now()), end() {}

    void reset() {
        start = std::chrono::high_resolution_clock::now();
    }

    double stop() {
        end = std::chrono::high_resolution_clock::now();
        return elapsed_seconds();
    }

    double elapsed_seconds() const {
        auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        return time_span.count();
    }
};

#endif //MAPFUA_COMMON_HPP
