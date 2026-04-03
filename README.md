# Project_A

Embedded micromouse robotics stack focused on localization, measurement planning, and control on ESP32. 
Built around an ESP32 / ESP-IDF workflow, with logic for estimating the robot’s position from range readings 
and improving position estimation by selecting informative measurements efficiently.

## Highlights
- Localization pipeline based on multi-sensor distance readings
- Measurement planning aimed at improving position estimation with as few measurements as possible
- Control architecture includes motion model, PID logic, and estimation components
- Structured ESP-IDF project with reusable tooling and build flow

## Quickstart
### Prerequisites
- ESP-IDF
- CMake
- Python environment required by ESP-IDF

### Build
```bash
idf.py build
