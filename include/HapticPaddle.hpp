#pragma once

#include <MEL/Mechatronics/Robot.hpp>
#include <MEL/Daq/Input.hpp>
#include <MEL/Daq/Output.hpp>
#include <MEL/Mechatronics/Motor.hpp>
#include <MEL/Mechatronics/Amplifier.hpp>
#include <MEL/Mechatronics/VirtualVelocitySensor.hpp>
#include "HallSensor.hpp"

class HapticPaddle : public mel::Robot {

public:

    /// Constructor
    HapticPaddle(mel::DigitalOutput::Channel DO,
                 mel::AnalogOutput::Channel AO,
                 mel::AnalogInput::Channel AI);

    /// Interactively calibrates the hall effect sensor in the console
    void calibrate();

private:

    /// Overrides the default Robot::enable function with some custom logic
    bool on_enable();

private:
    mel::Amplifier amp_;
    mel::Motor motor_;
    HallSensor position_sensor_;
    mel::VirtualVelocitySensor velocity_sensor_;
};