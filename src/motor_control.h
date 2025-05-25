#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "path_planning.h"

class MotorController {
public:
    MotorController();
    void send_motor_command(const MotorCommand& command);
    void stop();
};

#endif