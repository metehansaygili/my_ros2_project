#ifndef PLANNER_UTILS_HPP
#define PLANNER_UTILS_HPP

#include <cstdint>

namespace planner {

    enum class State : int32_t {
        CENTERLINETRACK = 1,
        LATTICE = 2,
        ASTAR = 3,
        PARKING_ASTAR = 4
    };

} // namespace planner

#endif // PLANNER_UTILS_HPP