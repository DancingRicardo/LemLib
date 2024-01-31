#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/stdout.hpp"
#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/misc.h"
#include "Graphy/Grapher.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// left motors on ports 8, 20, and 19. Motors on ports 8 and 20 are reversed. Using blue gearbox
auto leftBottomMotors = lemlib::makeMotorGroup({-12, -13}, pros::v5::MotorGears::blue);
// right motors on ports 2, 11, and 13. Motor on port 13 is reversed. Using blue gearbox
auto rightBottomMotors = lemlib::makeMotorGroup({4, 3}, pros::v5::MotorGears::blue);

// left motors on ports 8, 20, and 19. Motors on ports 8 and 20 are reversed. Using blue gearbox
auto leftPTOMotors = lemlib::makeMotorGroup({11}, pros::v5::MotorGears::blue);
// right motors on ports 2, 11, and 13. Motor on port 13 is reversed. Using blue gearbox
auto rightPTOMotors = lemlib::makeMotorGroup({-1}, pros::v5::MotorGears::blue);

pros::Motor intakeMotor(9);
pros::Motor flywheelMotor(7);

pros::adi::DigitalOut horizLeftFlap('A');
pros::adi::DigitalOut horizRightFlap('B');
pros::adi::DigitalOut vertLeftFlap('C');
pros::adi::DigitalOut vertRightFlap('H');

pros::adi::DigitalOut ptoLeft('E');
pros::adi::DigitalOut ptoRight('F');

// Inertial Sensor on port 11
pros::Imu imu(10);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    imu.reset(true);
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

void turnTo(float degree) {

    lemlib::FAPID turnPID(0, 0, 315, 0, 190, "Turn PID");

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    turnPID.setExit(1, .5, 900, 400, 3000);

    float targetDistance = degree;

    while (!turnPID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage
        int motorVoltage = turnPID.update(targetDistance, imu.get_yaw());

        leftBottomMotors->move_voltage(motorVoltage);
        rightBottomMotors->move_voltage(-motorVoltage);
        leftPTOMotors->move_voltage(motorVoltage);
        rightPTOMotors->move_voltage(-motorVoltage);

        pros::delay(20);
    }

}

void driveTo(float distance) {

    graphy::AsyncGrapher grapher("Drive PID", 20);

    lemlib::FAPID drivePID(0, 0, 400, 0, 0, "Drive PID");

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    drivePID.setExit(3, .1, 1000, 500, 5000);

    float targetDistance = distance;

    leftBottomMotors->set_zero_position_all(0);
    rightBottomMotors->set_zero_position_all(0);
    leftPTOMotors->set_zero_position_all(0);
    rightPTOMotors->set_zero_position_all(0);

    grapher.addDataType("Actual Distance", pros::c::COLOR_CYAN);
    grapher.addDataType("Target Distance", pros::c::COLOR_RED);
    grapher.addDataType("Voltage", pros::c::COLOR_YELLOW);

    while (!drivePID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage

        float currentDistanceTraveled = ((leftBottomMotors->get_position(0) + rightBottomMotors->get_position()) / 2) *
                                        3.14159265 / 180 * 1.625 * .75;

        float motorVoltage = drivePID.update(targetDistance, currentDistanceTraveled);

        if (motorVoltage > 12000) motorVoltage = 12000;

        grapher.update("Actual Distance",
                       currentDistanceTraveled / (targetDistance * 2));
        grapher.update("Target Distance", (targetDistance / (targetDistance * 2)));
        grapher.update("Voltage", (motorVoltage / 12000));

        leftBottomMotors->move_voltage(motorVoltage);
        rightBottomMotors->move_voltage(motorVoltage);
        leftPTOMotors->move_voltage(motorVoltage);
        rightPTOMotors->move_voltage(motorVoltage);

        pros::delay(20);
    }

}


void threeBall() {
    /*======================== grab acorn under bar */

    leftBottomMotors->move_voltage(3000);
    rightBottomMotors->move_voltage(3000);
    leftPTOMotors->move_voltage(3000);
    rightPTOMotors->move_voltage(3000);

    pros::delay(500);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    /*======================== go backwards halfway*/

    leftBottomMotors->move_voltage(-5500);
    rightBottomMotors->move_voltage(-6000);
    leftPTOMotors->move_voltage(-5500);
    rightPTOMotors->move_voltage(-6000);

    pros::delay(2000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    pros::delay(500);

    /*========================*/

    turnTo(-20);

    /*========================*/

    leftBottomMotors->move_voltage(-9000);
    rightBottomMotors->move_voltage(-9000);
    leftPTOMotors->move_voltage(-9000);
    rightPTOMotors->move_voltage(-9000);

    pros::delay(3000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);
}

void sixBall() {

    // Intake acorn under bar

    driveTo(3);

    // Back up

    driveTo(-20);

    // Turn to face goal

    turnTo(-20);

    // Drive to goal, extend back flap

    driveTo(-10);

    // Go out of goal, preparing to turn towards leftmost acorn

    driveTo(5);
    
    // Turn towards leftmost acorn

    turnTo(110);

    // Drive to leftmost acorn, intake

    intakeMotor.move(127);
    driveTo(20);

    // Turn towards goal, outtake

    intakeMotor.move(0);
    turnTo(35);
    intakeMotor.move(-127);
    pros::delay(500);
    intakeMotor.move(0);

    // Turn towards middle acorn by barrier

    turnTo(-45);

    // Drive to middle acorn, intake

    intakeMotor.move(127);
    driveTo(20);
    intakeMotor.move(0);

    // Turn towards goal

    turnTo(90);

    // Drive to goal, extend flaps, outtake

    horizLeftFlap.set_value(true);
    horizRightFlap.set_value(true);
    driveTo(20);

    // Retract flaps, back up so auton actually counts

    horizLeftFlap.set_value(false);
    horizRightFlap.set_value(false);
    driveTo(-10);

    

}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    graphy::AsyncGrapher grapher("Drive PID", 20);

    lemlib::FAPID turnPID(0, 0, 315, 0, 190, "Turn PID");
    lemlib::FAPID drivePID(0, 0, 950, 0, 2000, "Drive PID");

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    turnPID.setExit(1, .5, 900, 400, 3000);
    drivePID.setExit(3, .1, 1000, 500, 5000);

    float targetDistance = 48.f;

    leftBottomMotors->set_zero_position_all(0);
    rightBottomMotors->set_zero_position_all(0);
    leftPTOMotors->set_zero_position_all(0);
    rightPTOMotors->set_zero_position_all(0);

    grapher.addDataType("Actual Distance", pros::c::COLOR_CYAN);
    grapher.addDataType("Target Distance", pros::c::COLOR_RED);
    grapher.addDataType("Voltage", pros::c::COLOR_YELLOW);

    grapher.startTask();

    while (!drivePID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage

        float currentDistanceTraveled = ((leftBottomMotors->get_position(0) + rightBottomMotors->get_position()) / 2) *
                                        3.14159265 / 180 * 1.625 * .75;

        float motorVoltage = drivePID.update(targetDistance, currentDistanceTraveled);

        if (motorVoltage > 12000) motorVoltage = 12000;

        grapher.update("Actual Distance",
                       currentDistanceTraveled / (targetDistance * 2));
        grapher.update("Target Distance", (targetDistance / (targetDistance * 2)));
        grapher.update("Voltage", (motorVoltage / 12000));

        leftBottomMotors->move_voltage(motorVoltage);
        rightBottomMotors->move_voltage(motorVoltage);
        leftPTOMotors->move_voltage(motorVoltage);
        rightPTOMotors->move_voltage(motorVoltage);

        pros::delay(20);
    }

    while (1) {

    }

    // threeBall();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors

    bool isLifted = false;

    while (true) {

        /*=================*/

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            horizLeftFlap.set_value(true);
            horizRightFlap.set_value(true);
        } else {
            horizLeftFlap.set_value(false);
            horizRightFlap.set_value(false);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            vertLeftFlap.set_value(true);
            vertRightFlap.set_value(true);
        } else {
            vertLeftFlap.set_value(false);
            vertRightFlap.set_value(false);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeMotor.move_velocity(12000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeMotor.move_velocity(-12000);
        } else {
            intakeMotor.move_velocity(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            flywheelMotor.move_velocity(12000);
        } else {
            flywheelMotor.move_velocity(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            ptoLeft.set_value(false);
            ptoRight.set_value(false);
            isLifted = true;
        }   
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            ptoLeft.set_value(true);
            ptoRight.set_value(true);
            isLifted = false;
        }


        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && isLifted) {
            leftPTOMotors->move(-127);
            rightPTOMotors->move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && isLifted) {
            leftPTOMotors->move(127);
            rightPTOMotors->move(127);
        }
        else if (!isLifted) {
            leftPTOMotors->move(leftY);
            rightPTOMotors->move(rightY);
        }
        else {
            leftPTOMotors->move(0);
            rightPTOMotors->move(0);
        
        }


        /*===== DRIVE =====*/

        leftBottomMotors->move(leftY);
        rightBottomMotors->move(rightY);


        pros::delay(20);
    }
}