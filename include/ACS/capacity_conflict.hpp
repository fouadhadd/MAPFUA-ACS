#ifndef MAPFUA_CAPACITY_CONFLICT_HPP
#define MAPFUA_CAPACITY_CONFLICT_HPP

#include "../common.hpp"

struct CapacityConflict{
    int assigned_id;
    Location location1;
    Location location2;
    int time;

    CapacityConflict(int assigned_id, Location location1, Location location2, int time)
        : assigned_id(assigned_id), location1(location1), location2(location2), time(time) {}

    friend std::ostream &operator<<(std::ostream &os, const CapacityConflict &c) {
        if (c.location1 == c.location2) {
            return os << "CapacityConflict: Assigned Agent " << c.assigned_id
                      << " blocked vertex: " << c.location1
                      << " at time: " << c.time;
        } else {
            return os << "CapacityConflict: Assigned Agent " << c.assigned_id
                      << " blocked edge: " << c.location1 << "->" << c.location2
                      << " at time: " << c.time;
        }
    }
};

#endif //MAPFUA_CAPACITY_CONFLICT_HPP
