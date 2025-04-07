#pragma once

#include "tic.h"
#include "Eigen/Dense"
#define TAU (2*M_PI)

/* Gear train confifuration */
#define X_GEAR_RATIO (1.0f) // axel teeth : stepper shaft teeth = 12.9mm : 12.9mm
#define Y_GEAR_RATIO (1.0f) // axel teeth : stepper shaft teeth
#define Z_GEAR_RATIO (1.0f) // axel teeth : stepper shaft teeth
#define PULLEY_PITCH_RADIUS_MM (12.9f/2.0f) // effective radius of pulley in millimeters (12.9/2)

using namespace Eigen;

static const double z_offset_from_cor = -(85.8+ 70.0f/2.0f)*1e-3; // closest to bottom plate we can get. 71 = distance from CoR to limit switch trigger point


/* Motor mapping functions*/
Vector3d ConvertMotorPositionToMassPosition(int32_t x, int32_t y, int32_t z);
Vector3d ConvertMotorSpeedToMassVelocity(int32_t xdot, int32_t ydot, int32_t zdot );
std::vector<int32_t> ConvertMassPositionToMotorPosition(double x_pos, double y_pos, double z_pos);
