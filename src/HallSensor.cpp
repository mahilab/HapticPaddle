#include "HallSensor.hpp"

using namespace mel;

HallSensor::HallSensor(const std::string& name,
                       mel::AnalogInput::Channel ai,
                       double gain,
                       double offset) :
    PositionSensor(name), ai_(ai), gain_(gain), offset_(offset)
{

}

double HallSensor::get_position() {
    position_ = ai_.get_value() * gain_ + offset_;
    return position_;
}

bool HallSensor::on_enable() { 
    return true;
}

bool HallSensor::on_disable() { 
    return true; 
}

