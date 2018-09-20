#include "HapticPaddle.hpp"
#include <MEL/Core/Console.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>
#include <fstream>

using namespace mel;

HapticPaddle::HapticPaddle(mel::DigitalOutput::Channel d_o,
                           mel::AnalogOutput::Channel ao,
                           mel::AnalogInput::Channel ai) :
    // Robot constructor
    Robot("haptic_paddle"),
    // init amplifier
    amp_("amc_12a8", High, d_o, -1.065, ao),
    // init motor
    motor_("pitman_9434", 0.0229, amp_, Limiter(1.8821, 12.0, seconds(1))),
    // init position sensor
    position_sensor_("honeywell_ss49et", ai),
    // init virtual velocity sensor
    velocity_sensor_("honeywell_ss49et", position_sensor_)
{
    // create joint
    add_joint(mel::Joint("paddle_joint_0", &motor_, 0.713 / 6.250, &position_sensor_, 1.0,
        &velocity_sensor_, 1.0, { -50 * DEG2RAD, 50 * DEG2RAD },
        500 * DEG2RAD, 1.0));
}

void HapticPaddle::calibrate() {
    std::ofstream file;
    file.open("calibration.txt");
    std::vector<double> positions = { -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30 };
    print("\nRotate to the indicated position when prompted and hold. Press ENTER to begin calibration.\n", Color::Yellow);
    std::vector<double> volts(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        beep();
        std::cout << std::setfill(' ') << std::setw(4) << positions[i] << "  :  ";
        sleep(seconds(3));
        position_sensor_.ai_.update();
        volts[i] = position_sensor_.ai_.get_value();
        std::cout << volts[i] << " V \n";
    }
    std::cout << "\n";
    std::vector<double> mb = linear_regression(volts, positions);
    position_sensor_.gain_ = mb[0];
    position_sensor_.offset_ = mb[1];
    file << mb[0] << "\n";
    file << mb[1] << "\n";
    LOG(Info) << "Calibrated Haptic Paddle hall effect sensor";
    file.close();
}

bool HapticPaddle::on_enable() {
    // load calibration
    std::ifstream file;
    file.open("calibration.txt");
    if (file.is_open()) {
        // read in previous calibration
        double gain, offset;
        file >> gain >> offset;
        position_sensor_.offset_ = offset * DEG2RAD;
        position_sensor_.gain_ = gain * DEG2RAD;
        LOG(Info) << "Imported Haptic Paddle hall effect sensor calibration";
        file.close();
    }
    else {
        file.close();
        calibrate();
    }
    return true;
}