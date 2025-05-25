#include "motor_control.h"
#include <iostream>

MotorController::MotorController() {
    std::cout << "MotorController initialized\n";
}

void MotorController::send_motor_command(const MotorCommand& command) {
    if (command.duration <= 0.0) return;

    std::cout << "Linear Velocity: " << command.linear_velocity
              << ", Angular Velocity: " << command.angular_velocity
              << ", Duration: " << command.duration << "s\n";
}

void MotorController::stop() {
    std::cout << "Motors stopped\n";
}