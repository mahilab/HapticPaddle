#pragma once

#include <MEL/Daq/Input.hpp>
#include <MEL/Mechatronics/PositionSensor.hpp>

class HallSensor : public mel::PositionSensor {
public:
    /// Constructor
    HallSensor(const std::string& name,
               mel::AnalogInput::Channel ai,
               double gain   = 0.0,
               double offset = 0.0);

    /// Gets the position of the hall effect sensor in [rad]
    double get_position() override;

private:

    bool on_enable() override;
    bool on_disable() override;

public:
    mel::AnalogInput::Channel ai_;  ///< voltage read from hall effect sensor
    double gain_;                   ///< calibration gain   [rad/V]
    double offset_;                 ///< calibration offset [rad]
};
