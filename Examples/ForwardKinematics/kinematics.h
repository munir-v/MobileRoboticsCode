#ifndef DIFFERENTIALDRIVEROBOT_H
#define DIFFERENTIALDRIVEROBOT_H

#include <cmath>  // For sin, cos, and fmod

class DifferentialDriveRobot {
    // Private variables to store global state
private:
    double x_g;   // Global x-coordinate
    double y_g;   // Global y-coordinate
    double θ_g;   // Global orientation (in radians)
    double L;     // Wheelbase (distance between wheels in cm)

 public:
    // Constructor initializes robot state
    DifferentialDriveRobot(double wheelbase) 
        : x_g(0.0), y_g(0.0), θ_g(0.0), L(wheelbase) {}

    // Setup function (like in the Display class)
    void setup() {
        x_g = 0.0;
        y_g = 0.0;
        θ_g = 0.0;
    }

    // Helper function to normalize angle to [-π, π]
    double normalize_angle(double angle) {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    // Forward kinematics to update position and orientation
    void forward_kinematics(double v_L, double v_R, double delta_t) {
        double v = (v_L + v_R) / 2.0;  // Linear velocity
        double ω = (v_R - v_L) / L;    // Angular velocity

        double Δθ = ω * delta_t;             // Change in orientation
        θ_g = normalize_angle(θ_g + Δθ);

        double Δx = v * cos(θ_g) * delta_t;  // Change in x-coordinate
        double Δy = v * sin(θ_g) * delta_t;  // Change in y-coordinate

        x_g += Δx;  // Update global x-coordinate
        y_g += Δy;  // Update global y-coordinate
    }

    // Function to get the current position and orientation
    void get_position(double &x, double &y, double &theta) const {
        x = x_g;
        y = y_g;
        theta = θ_g;
    }

    // Loop step function (placeholder for future use)
    void loopStep() {
    }
};

#endif  // DIFFERENTIALDRIVEROBOT_H
