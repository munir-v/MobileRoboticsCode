#ifndef DIFFERENTIALDRIVEROBOT_H
#define DIFFERENTIALDRIVEROBOT_H

#include <iostream>
#include <cmath>  // For sin, cos, and fmod

class DifferentialDriveRobot {
private:
    // Global position and orientation
    double x_g;   // Global x-coordinate
    double y_g;   // Global y-coordinate
    double θ_g;   // Global orientation (in radians)

    // Robot state variables
    double L;     // Distance between wheels (wheelbase in cm)
    double v_L = 0.0;
    double v_R = 0.0;
    double t = 0.0; 

public:
    DifferentialDriveRobot(double wheelbase) 
        : x_g(0.0), y_g(0.0), θ_g(0.0), L(wheelbase) {}

    // Function to normalize the angle to the range [-π, π]
    double normalize_angle(double angle) {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    // Forward kinematics function to update position and orientation
    void forward_kinematics(double v_L, double v_R, double t) {
        // Calculate linear and angular velocities
        double v = (v_L + v_R) / 2.0;           // Linear velocity
        double ω = (v_R - v_L) / L;             // Angular velocity

        // Compute the change in orientation
        double Δθ = ω * t;
        θ_g += Δθ;  // Update global orientation

        // Calculate the change in global position
        double Δx = v * cos(θ_g) * t;  // Change in x-coordinate
        double Δy = v * sin(θ_g) * t;  // Change in y-coordinate

        // Update the global position
        x_g += Δx;
        y_g += Δy;

        // Normalize the orientation
        θ_g = normalize_angle(θ_g);
    }
};

#endif 