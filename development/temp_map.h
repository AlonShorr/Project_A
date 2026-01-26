#ifndef MAIN_TEMP_MAP_H
#define MAIN_TEMP_MAP_H

#include <misc_utils/physical_size.h>
#include "position.h"
#include "segment.h"
#include <array>

namespace micromouse
{

inline constexpr auto wall_length = unit_cast<meters>(18_cm).count();

// Map Dimensions: 9 cells wide (162cm) x 3 cells high (54cm) 
// Note: 9 cells high * 18cm = 162cm
inline const std::array<Segment, 4> maze_map = {
    // How to define segments:
    // Segment{point1, point2]} where:
    // point1 = Eigen::Vector2f(x1, y1), point2 = Eigen::Vector2f(x2, y2)

    // Left Wall: p1=(0,0), p2=(0,3)
    Segment{Eigen::Vector2f{0.0f, 0.0f}, Eigen::Vector2f{0.0f, 3 * wall_length}},
    
    // Top Wall: p1=(0,0), p2=(9,0)
    Segment{Eigen::Vector2f{0.0f, 0.0f}, Eigen::Vector2f{9 * wall_length, 0.0f}},
    
    // Right Wall: p1=(9,0), p2=(9,3)
    Segment{Eigen::Vector2f{9 * wall_length, 0.0f}, Eigen::Vector2f{9 * wall_length, 3 * wall_length}},
    
    // Bottom Wall: p1=(0,3), p2=(9,3)
    Segment{Eigen::Vector2f{0.0f, 3 * wall_length}, Eigen::Vector2f{9 * wall_length, 3 * wall_length}},
};
}  // namespace micromouse

#endif  // MAIN_TEMP_MAP_H