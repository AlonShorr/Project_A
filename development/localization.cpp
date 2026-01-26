#include "localization.h"
#include <limits>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace micromouse {

// Sensor Configuration
// X = Forward, Y = Left (Standard ROS convention relative to center)
// Disposition of the sensors on the robot (add later)
// const float SENSOR_X[] = {0.062f, 0.090f, 0.094f, 0.092f, 0.067f};
// const float SENSOR_Y[] = {0.054f, 0.040f, 0.0f, -0.039f, -0.054f};
const float SENSOR_THETA[] = {-1.5707f, -0.6495f, 0.0f, 0.7037f, 1.5707f}; // radians

Localization::Localization() {
    float p = 1.0f / (LOC_ROWS * LOC_COLS); // initialize with uniform distribution 1/(N)
    for(int r=0; r<LOC_ROWS; r++) {
        for(int c=0; c<LOC_COLS; c++) {
            m_probs[r][c] = p;
        }
    }
}

void Localization::init(const std::vector<segment::Segment>& segments) {
    m_logger.init();
    // m_logger.log("Localization Init...\n");
    std::printf("Localization Init...\n");

    m_map_segments = segments;

    // Pre-compute Lookup Table
    for(int r=0; r<LOC_ROWS; r++) {
        for(int c=0; c<LOC_COLS; c++) {

            float cx = (c + 0.5f) * CELL_SIZE;
            float cy = (r + 0.5f) * CELL_SIZE;

            for(int a=0; a<LOC_ANGLES; a++) {
                float robot_theta = a * (2.0f * 3.14159f / 8.0f);
                for(int s=0; s<LOC_SENSORS; s++) {
                    float dist = cast_ray(cx, cy, robot_theta + SENSOR_THETA[s]);
                    m_lookup[r][c][a][s] = dist; // in meters
                }
            }
        }
    }
    std::printf("Localization Init Done.\n");
    // m_logger.log("Init Done.\n");
}

void Localization::print_distance_map_debug() {
    // m_logger.log("\n=== DISTANCE MAP DUMP ===\n");
    std::printf("\n=== DISTANCE MAP DUMP ===\n");
    // Format: Cell (r,c): Angle 
    for (int r=0; r<LOC_ROWS; r++) {
        for(int c=0; c<LOC_COLS; c++) {
            std::printf("Cell (%d,%d):\n", r, c);
            for(int a=0; a<LOC_ANGLES; a++) {
                std::printf("  Ang %d (%d°): {%.2f, %.2f, %.2f, %.2f, %.2f},\n", 
                             a,
                             a*45,
                             m_lookup[r][c][a][0],
                             m_lookup[r][c][a][1],
                             m_lookup[r][c][a][2],
                             m_lookup[r][c][a][3],
                             m_lookup[r][c][a][4]);
            }
        }
    }
    std::printf("=== END DISTANCE MAP DUMP ===\n\n");
    // m_logger.log("=== END DISTANCE MAP DUMP ===\n\n");
}

void Localization::network_loop() {
    m_logger.handle();
}

float Localization::cast_ray(float start_x, float start_y, float angle) {
    float min_dist = 1.3f; // Max range 1.3m
    float dx = std::cos(angle);
    float dy = std::sin(angle);
    float end_x = start_x + dx * min_dist;
    float end_y = start_y + dy * min_dist;
    
    segment::Segment ray(Eigen::Vector2f(start_x, start_y), Eigen::Vector2f(end_x, end_y));

    for(const auto& wall : m_map_segments) {
        float d = ray.intersection_distance(wall);
        if (d < min_dist) min_dist = d;
    }
    return min_dist;
}

void Localization::update(const std::array<float, LOC_SENSORS>& measurements, float current_yaw) {
    network_loop();

    // --- LOG SENSORS TO PHONE ---
    // Only log every 500ms to avoid flooding
    static unsigned long last_sensor_log = 0;
    if (millis() - last_sensor_log > 500) {
        m_logger.log("SENSORS: [%.3f, %.3f, %.3f, %.3f, %.3f]\n", 
                     measurements[0], measurements[1], measurements[2], measurements[3], measurements[4]);
        last_sensor_log = millis();
    }
    // ----------------------------

    // Normalize Yaw
    while(current_yaw < 0) current_yaw += 2 * 3.14159f;
    while(current_yaw >= 2 * 3.14159f) current_yaw -= 2 * 3.14159f;
    
    int angle_idx = static_cast<int>(std::round(current_yaw / (3.14159f / 4.0f))) % 8;

    for(int r=0; r<LOC_ROWS; r++) {
        for(int c=0; c<LOC_COLS; c++) {
            if(m_probs[r][c] < 0.00001f) continue;

            float likelihood = 1.0f;
            for(int s=0; s<LOC_SENSORS; s++) {

                float z = measurements[s]; // real readings 

                // Ignore invalid readings
                if(z > 1.3f || z < 0.001f) continue;

                float expected = m_lookup[r][c][angle_idx][s];
                float diff = z - expected;
                float sigma = 0.02f; // 2cm standard deviation 
                
                likelihood *= std::exp(-(diff*diff)/(2*sigma*sigma));
            }
            m_probs[r][c] *= likelihood;
        }
    }
    normalize();
}

void Localization::normalize() {
    float sum = 0.0f;
    float max_p = -1.0f;
    int best_r = 0, best_c = 0;

    for(int r=0; r<LOC_ROWS; r++) {
        for(int c=0; c<LOC_COLS; c++) {
            sum += m_probs[r][c];
        }
    }

    if(sum > 0) {
        for(int r=0; r<LOC_ROWS; r++) {
            for(int c=0; c<LOC_COLS; c++) {
                m_probs[r][c] /= sum;
                if(m_probs[r][c] > max_p) {
                    max_p = m_probs[r][c];
                    best_r = r; best_c = c;
                }
            }
        }
        
        // Log Best Position
        static unsigned long last_pos_log = 0;
        if(millis() - last_pos_log > 500) {
            m_logger.log("Best: (%d, %d) Prob: %.2f\n", best_r, best_c, max_p);
            last_pos_log = millis();
        }
    } else {
        // Reset if lost
        float p = 1.0f / (LOC_ROWS * LOC_COLS);
        for(int r=0; r<LOC_ROWS; r++) {
            for(int c=0; c<LOC_COLS; c++) {
                m_probs[r][c] = p;
            }
        }
        
        static unsigned long last_lost_log = 0;
        if(millis() - last_lost_log > 1000) {
            m_logger.log("Lost. Resetting.\n");
            last_lost_log = millis();
        }
    }
}

void Localization::get_best_position(int& out_row, int& out_col, float& out_prob) {
    float max_p = -1.0f;
    for(int r=0; r<LOC_ROWS; r++) {
        for(int c=0; c<LOC_COLS; c++) {
            if(m_probs[r][c] > max_p) {
                max_p = m_probs[r][c];
                out_row = r; out_col = c;
            }
        }
    }
    out_prob = max_p;
}

}