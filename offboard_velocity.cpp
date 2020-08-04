/**
 * @file offboard_velocity.cpp
 * @brief Example that demonstrates offboard velocity control in local NED and
 * body coordinates
 *
 * @authors Author: Julian Oes <julian@oes.ch>,
 *                  Shakthi Prashanth <shakthi.prashanth.m@intel.com>
 */

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string& message)
{
    if (result != Action::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string& message)
{
    if (result != Offboard::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string& message)
{
    if (result != ConnectionResult::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string& offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using NED co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log
 * otherwise.
 */
bool offb_ctrl_ned(std::shared_ptr<mavsdk::Offboard> offboard)
{
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    const Offboard::VelocityNedYaw stay{};
    offboard->set_velocity_ned(stay);

    // TODO: Try to wait user sel offboard mode
    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

    offboard_log(offb_mode, "Turn to face East");

    Offboard::VelocityNedYaw turn_east{};
    turn_east.yaw_deg = 90.0f;
    offboard->set_velocity_ned(turn_east);
    sleep_for(seconds(1)); // Let yaw settle.


    offboard_log(offb_mode, "Turn to face West");
    Offboard::VelocityNedYaw turn_west{};
    turn_west.yaw_deg = 270.0f;
    offboard->set_velocity_ned(turn_west);
    sleep_for(seconds(2));


    // Now, stop offboard mode.
    offboard_result = offboard->stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
}



void wait_until_discover(Mavsdk& dc)
{
    std::cout << "Waiting to discover system..." << std::endl;
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    dc.register_on_discover([&discover_promise](uint64_t uuid) {
        std::cout << "Discovered system with UUID: " << uuid << std::endl;
        discover_promise.set_value();
    });

    discover_future.wait();
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

Telemetry::LandedStateCallback
landed_state_callback(std::shared_ptr<Telemetry>& telemetry, std::promise<void>& landed_promise)
{
    return [&landed_promise, &telemetry](Telemetry::LandedState landed) {
        switch (landed) {
            case Telemetry::LandedState::OnGround:
                std::cout << "On ground" << std::endl;
                break;
            case Telemetry::LandedState::TakingOff:
                std::cout << "Taking off..." << std::endl;
                break;
            case Telemetry::LandedState::Landing:
                std::cout << "Landing..." << std::endl;
                break;
            case Telemetry::LandedState::InAir:
                std::cout << "Taking off has finished." << std::endl;
                telemetry->subscribe_landed_state(nullptr);
                landed_promise.set_value();
                break;
            case Telemetry::LandedState::Unknown:
                std::cout << "Unknown landed state." << std::endl;
                break;
        }
    };
}

int main(int argc, char** argv)
{
    Mavsdk dc;
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Wait for the system to connect via heartbeat
    wait_until_discover(dc);

    // System got discovered.
    System& system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    while (!telemetry->health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "System is ready" << std::endl;

    std::promise<void> in_air_promise;
    auto in_air_future = in_air_promise.get_future();

    Action::Result arm_result = action->arm();
    action_error_exit(arm_result, "Arming failed");
    std::cout << "Armed" << std::endl;

    Action::Result takeoff_result = action->takeoff();
    action_error_exit(takeoff_result, "Takeoff failed");

    telemetry->subscribe_landed_state(landed_state_callback(telemetry, in_air_promise));
    in_air_future.wait();


    //  using local NED co-ordinates
    ret = offb_ctrl_ned(offboard);
    if (ret == false) {
        return EXIT_FAILURE;
    }


    const Action::Result land_result = action->land();
    action_error_exit(land_result, "Landing failed");

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for
    // a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}