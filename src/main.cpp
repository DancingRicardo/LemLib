#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/eventhandler/testevents.hpp"
#include "lemlib/logger/stdout.hpp"
#include "pros/misc.h"
#include "lemlib/eventhandler/eventhandler.hpp"
#include "lemlib/eventhandler/prosevents/buttonevents.hpp"
#include "lemlib/devices/gamepad/prosgamepad.hpp"
#include "pros/misc.hpp"
#include <memory>
#include <vector>

/*
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// left motors on ports 8, 20, and 19. Motors on ports 8 and 20 are reversed. Using blue gearbox
auto leftMotors = lemlib::makeMotorGroup({-8, -20, 19}, pros::v5::MotorGears::blue);
// right motors on ports 2, 11, and 13. Motor on port 13 is reversed. Using blue gearbox
auto rightMotors = lemlib::makeMotorGroup({2, 11, -13}, pros::v5::MotorGears::blue);

// Inertial Sensor on port 11
pros::Imu imu(11);

// horizontal tracking wheel. Port 4, 2.75" diameter, 3.7" offset, back of the robot
lemlib::TrackingWheel horizontal(4, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(leftMotors, // left motor group
                              rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// linear motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            30, // derivative gain (kD)
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             10, // derivative gain (kD)
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             20 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders, so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to nullptr as we don't have one
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have one
                            &horizontal, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Differential chassis(drivetrain, linearController, angularController, sensors);
*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {}

/**
 * Runs in driver control
 */
void opcontrol() {
    std::shared_ptr<pros::Controller> controlla = std::make_shared<pros::Controller>(pros::E_CONTROLLER_MASTER);

    lemlib::PROSButtonEvent XEvent(controlla, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_X);
    lemlib::PROSButtonEvent BEvent(controlla, pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_B);
    lemlib::PROSButtonEvent YEvent(controlla, pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_Y);
    lemlib::PROSButtonEvent AEvent(controlla, pros::E_CONTROLLER_DIGITAL_A, pros::E_CONTROLLER_DIGITAL_A);
    lemlib::PROSButtonEvent L1Event(controlla, pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L1);
    lemlib::PROSButtonEvent L2Event(controlla, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_L2);
    lemlib::PROSButtonEvent R1Event(controlla, pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R1);
    lemlib::PROSButtonEvent R2Event(controlla, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R2);

    std::vector<std::shared_ptr<lemlib::Event>> buttonsEvents(
        {std::make_shared<lemlib::PROSButtonEvent>(L1Event), std::make_shared<lemlib::PROSButtonEvent>(L2Event),
         std::make_shared<lemlib::PROSButtonEvent>(BEvent), std::make_shared<lemlib::PROSButtonEvent>(YEvent),
         std::make_shared<lemlib::PROSButtonEvent>(L1Event), std::make_shared<lemlib::PROSButtonEvent>(L2Event),
         std::make_shared<lemlib::PROSButtonEvent>(R1Event), std::make_shared<lemlib::PROSButtonEvent>(R2Event)});

    lemlib::EventHandler buttonEventHandler(buttonsEvents);

    lemlib::TESTEvent testEventA(false, 0);
    lemlib::TESTEvent testEventB(false, 1);
    lemlib::TESTEvent testEventC(true, 2);
    lemlib::TESTEvent testEventD(false, 3);
    lemlib::TESTEvent testEventE(true, 4);

    std::vector<std::shared_ptr<lemlib::Event>> testEvents(
        {std::make_shared<lemlib::TESTEvent>(testEventA), std::make_shared<lemlib::TESTEvent>(testEventB),
         std::make_shared<lemlib::TESTEvent>(testEventC), std::make_shared<lemlib::TESTEvent>(testEventD),
         std::make_shared<lemlib::TESTEvent>(testEventE)});

    lemlib::EventHandler testEventHandler(testEvents);

    std::cout << "Started" << std::endl;

    lemlib::PROS_Gamepad gamepad(pros::E_CONTROLLER_MASTER, {"Driver", "Debug"}, "Driver",
                                 std::make_unique<lemlib::EventHandler>(buttonEventHandler));
    
    gamepad.startMainLoop();
    
    // controller
    // loop to continuously update motors
    while (true) {
        if (testEventHandler.checkEvent(0)) { std::cout << " Event A triggered. " << std::endl; }

        if (testEventHandler.checkEvent(1)) { std::cout << " Event B triggered. " << std::endl; }

        if (testEventHandler.checkEvent(2)) { std::cout << " Event C triggered. " << std::endl; }

        if (testEventHandler.checkEvent(3)) { std::cout << " Event D triggered. " << std::endl; }

        if (testEventHandler.checkEvent(4)) { std::cout << " Event E triggered. " << std::endl; }

        std::cout << "Looped through" << std::endl;

        // move the chassis with curvature drive
        // delay to save resources
        pros::delay(10);
    }
}