#include <MEL/Core/Console.hpp>
#include <MEL/Math.hpp>
#include <MEL/Core.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Communications.hpp>
#include <MEL/Mechatronics/PdController.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Logging/Log.hpp>
#include "HapticPaddle.hpp"

using namespace mel;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    if (event == CtrlEvent::CtrlC) {
        print("Ctrl+C Pressed!");
        stop = true;
    }
    return true;
}

//==============================================================================
// TUNNEL DEMO
//==============================================================================

Waveform tunnel_trajectory = Waveform(Waveform::Sin, seconds(2), 30.0 * DEG2RAD);
PdController tunnel_pd = PdController(1.0, 0.01);

double tunnel(double position, double velocity, Time current_time) {
    double x_ref = tunnel_trajectory.evaluate(current_time);
    return tunnel_pd.calculate(x_ref, position, 0, velocity);
}

//==============================================================================
// WALL DEMO
//==============================================================================

double wall_position = 0;
PdController wall_into_pd = PdController(5.0, 0.1);
PdController wall_outof_pd = PdController(5.0, 0.0);

double wall(double position, double velocity) {
    if (position > wall_position) {
        if (velocity > 0)
            return wall_into_pd.calculate(wall_position * DEG2RAD, position, 0, velocity);
        else
            return wall_outof_pd.calculate(wall_position * DEG2RAD, position, 0, velocity);
    }
    else
        return 0;
}

//==============================================================================
// NOTCHES DEMO
//==============================================================================

std::vector<double> notch_positions = { -30, -15, 0, 15, 30 };
PdController notch_pd = PdController(3.0, 0.05);

double notches(double position, double velocity) {
    for (std::size_t i = 0; i < notch_positions.size(); ++i) {
        if (position < ((notch_positions[i] + 7.5) * DEG2RAD) &&
            position >((notch_positions[i] - 7.5) * DEG2RAD)) {
            return notch_pd.calculate(notch_positions[i] * DEG2RAD, position, 0, velocity);
        }
    }
    return 0.0;
}

//==============================================================================
// MAIN FUNCTION
//==============================================================================

int main(int argc, char* argv[]) {

    // register CTRL-C handler
    register_ctrl_handler(handler);

    // make and parse console options
    Options options("haptic_paddle.exe", "MAHI Lab Haptic Paddle Suite");
    options.add_options()
        ("c,calibrate", "Calibrates Hall Effect Sensor")
        ("t,tunnel", "Tunnel demo")
        ("w,wall", "Wall demo")
        ("n,notches", "Notches demo")
        ("h,help", "Prints this Help Message");
    auto input = options.parse(argc, argv);

    // print help message if requested
    if (input.count("h")) {
        print(options.help());
        return 0;
    }

    // create MELShare for MELScope
    std::vector<double> ms_data(4);
    MelShare ms("to_ms");

    // create Q8 USB
    Q8Usb q8;

    // create Haptic Paddle
    HapticPaddle hp(q8.DO[0], q8.AO[0], q8.AI[0]);

    // open Q8 USB
    if (!q8.open())
        return 1;

    // enable Q8 USB
    if (!q8.enable())
        return 1;

    // enable haptic paddle
    prompt("Press ENTER to enable Haptic Paddle");
    hp.enable();

    // perform calibration if requested
    if (input.count("c")) {
        hp.calibrate();
        return 0;
    }

    // create control loop timer
    Timer timer(hertz(1000));

    // Butterworth filters for position and velocity
    Butterworth buttpos(4, hertz(25), timer.get_frequency());
    Butterworth buttvel(4, hertz(25), timer.get_frequency());
    double filtered_pos, filtered_vel;

    // torque
    double torque;

    // enter control loop
    prompt("Press ENTER to start control loop");
    timer.restart();
    while (!stop) {

        // update hardware
        q8.update_input();
        hp[0].get_velocity_sensor<VirtualVelocitySensor>()->update();

        // filter position and velocity
        filtered_pos = buttpos.update(hp[0].get_position());
        filtered_vel = buttvel.update(hp[0].get_velocity());

        // compute torque
        if (input.count("t"))
            torque = tunnel(hp[0].get_position(), filtered_vel, timer.get_elapsed_time_actual());
        else if (input.count("w"))
            torque = wall(hp[0].get_position(), filtered_vel);
        else if (input.count("n"))
            torque = notches(hp[0].get_position(), filtered_vel);
        else
            torque = 0;

        // set torque
        hp[0].set_torque(torque);

        // write MelShare for MEL Scope
        ms_data[0] = hp[0].get_position() * RAD2DEG;
        ms_data[1] = filtered_pos * RAD2DEG;
        ms_data[2] = torque;
        ms_data[3] = filtered_vel * RAD2DEG;
        ms.write_data(ms_data);

        // check limits
        if (hp.any_position_limit_exceeded() ||
            hp.any_torque_limit_exceeded() ||
            mel::abs(filtered_vel) > 500 * DEG2RAD)
        {
            LOG(Fatal) << "Haptic Paddle safety limit exceeded";
            stop = true;
        }

        // update hardware output
        q8.update_output();

        // wait timer
        timer.wait();
    }

    return 0;
}

