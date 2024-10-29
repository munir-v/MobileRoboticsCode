#ifndef DIFFERENTIALDRIVEROBOT_H
#define DIFFERENTIALDRIVEROBOT_H

#include <cmath> // For sin, cos, and fmod

class DifferentialDriveRobot
{
    // Private variables to store global state
private:
    double x_g;     // Global x-coordinate
    double y_g;     // Global y-coordinate
    double theta_g; // Global orientation (in radians)
    double L;       // Wheelbase (distance between wheels in cm)

public:
    // Constructor initializes robot state
    DifferentialDriveRobot(double wheelbase)
        : x_g(0.0), y_g(0.0), theta_g(0.0), L(wheelbase) {}

    // Setup function (like in the Display class)
    void setup()
    {
    }

    // Helper function to normalize angle to [-π, π]
    double normalize_angle(double angle)
    {
        return fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    // Forward kinematics to update position and orientation
    void forward_kinematics(double v_L, double v_R, double delta_t)
    {
        double v = (v_L + v_R) / 2.0;   // Linear velocity
        double omega = (v_R - v_L) / L; // Angular velocity

        // Serial.print("v: ");
        // Serial.print(v);
        // Serial.print(", omega: ");
        // Serial.println(omega);

        double delta_theta = omega * delta_t; // Change in orientation
        theta_g = normalize_angle(theta_g + delta_theta);

        // Serial.print("delta_theta: ");
        // Serial.print(delta_theta);
        // Serial.print("theta: ");
        // Serial.println(theta_g);

        double delta_x = v * cos(theta_g) * delta_t; // Change in x-coordinate
        double delta_y = v * sin(theta_g) * delta_t; // Change in y-coordinate

        // Serial.print("delta_x: ");
        // Serial.print(delta_x);
        // Serial.print(", delta_y: ");
        // Serial.println(delta_y);

        x_g += delta_x; // Update global x-coordinate
        y_g += delta_y; // Update global y-coordinate
    }

    double get_x()
    {
        return x_g;
    }

    double get_y()
    {
        return y_g;
    }

    double get_theta()
    {
        return theta_g;
    }

    // Loop step function (placeholder for future use)
    void loopStep()
    {
    }
};

#endif // DIFFERENTIALDRIVEROBOT_H
