#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <vector>
#include <array>
#include <cmath>
#include "segment.h"
#include "wifi_logger.h"

namespace micromouse {

// UPDATED MAP DIMENSIONS
static constexpr int LOC_COLS = 9; // 162cm / 18cm = 9 cells
static constexpr int LOC_ROWS = 3; // 54cm / 18cm = 3 cells

static constexpr int LOC_ANGLES = 8;
static constexpr int LOC_SENSORS = 5;
static constexpr float CELL_SIZE = 0.18f;

class Localization {
public:
    Localization();

    void init(const std::vector<segment::Segment>& segments);
    void update(const std::array<float, LOC_SENSORS>& measurements, float current_yaw);
    void print_distance_map_debug();
    void get_best_position(int& out_row, int& out_col, float& out_prob);
    void network_loop(); 

private:
    WifiLogger m_logger; 

    std::vector<segment::Segment> m_map_segments;
    float m_probs[LOC_ROWS][LOC_COLS];
    float m_lookup[LOC_ROWS][LOC_COLS][LOC_ANGLES][LOC_SENSORS]; // distance lookup table

    float cast_ray(float start_x, float start_y, float angle);
    void normalize();
};

}

#endif