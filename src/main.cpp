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
auto leftPTOMotors = lemlib::makeMotorGroup({11}, pros::v5::MotorGears::blue);
auto rightPTOMotors = lemlib::makeMotorGroup({-1}, pros::v5::MotorGears::blue);

// left motors on ports 8, 20, and 19. Motors on ports 8 and 20 are reversed. Using blue gearbox
auto leftBottomMotors = lemlib::makeMotorGroup({-12, -13}, pros::v5::MotorGears::blue);
// right motors on ports 2, 11, and 13. Motor on port 13 is reversed. Using blue gearbox
auto rightBottomMotors = lemlib::makeMotorGroup({4, 3}, pros::v5::MotorGears::blue);

pros::Imu imu(10);

pros::Motor intakeMotor(9);
pros::Motor flywheelMotor(7);

pros::adi::DigitalOut horizLeftFlap('A');
pros::adi::DigitalOut horizRightFlap('B');
pros::adi::DigitalOut vertLeftFlap('C');
pros::adi::DigitalOut vertRightFlap('H');

pros::adi::DigitalOut ptoLeft('E');
pros::adi::DigitalOut ptoRight('F');

void turnTo(float degree) {

    lemlib::FAPID turnPID(0, 0, 315, 0, 190, "Turn PID");

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    turnPID.setExit(1, .5, 900, 400, 3000);

    float targetDistance = degree + imu.get_rotation();

    while (!turnPID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage
        int motorVoltage = turnPID.update(targetDistance, imu.get_rotation());

        leftBottomMotors->move_voltage(motorVoltage);
        rightBottomMotors->move_voltage(-motorVoltage);
        leftPTOMotors->move_voltage(motorVoltage);
        rightPTOMotors->move_voltage(-motorVoltage);

        pros::delay(20);
    }

}

void driveTo(float distance) {

    graphy::AsyncGrapher grapher("Drive PID", 20);

    lemlib::FAPID drivePID(0, 0, 1000, 0, 4750, "Drive PID"); // 2000
    lemlib::FAPID straightPID(0, 0, 150, 0, 300, "Turn PID");

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    drivePID.setExit(3, .1, 1000, 500, 5000);
    straightPID.setExit(1, .5, 900, 400, 5000);
    float targetDistance = distance;
    float startDegree = imu.get_yaw();

    leftBottomMotors->set_zero_position_all(0);
    rightBottomMotors->set_zero_position_all(0);
    leftPTOMotors->set_zero_position_all(0);
    rightPTOMotors->set_zero_position_all(0);

    grapher.addDataType("Actual Yaw", pros::c::COLOR_CYAN);
    grapher.addDataType("Target Yaw", pros::c::COLOR_RED);
    grapher.addDataType("Voltage", pros::c::COLOR_YELLOW);

    grapher.startTask();

    while (!drivePID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage

        float currentDistanceTraveled = ((leftBottomMotors->get_position(0) + rightBottomMotors->get_position()) / 2) *
                                        3.14159265 / 180 * 1.625 * .75;

        float motorVoltage = drivePID.update(targetDistance, currentDistanceTraveled);
        float straightVoltage = straightPID.update(startDegree, imu.get_yaw());

        if (motorVoltage > 12000) motorVoltage = 12000;

        grapher.update("Actual Yaw", (imu.get_yaw()) / (startDegree * 2));
        grapher.update("Target Yaw", (startDegree / (startDegree * 2)));
        grapher.update("Voltage", (straightVoltage / 12000));

        leftBottomMotors->move_voltage(motorVoltage + straightVoltage);
        rightBottomMotors->move_voltage(motorVoltage - straightVoltage);
        leftPTOMotors->move_voltage(motorVoltage + straightVoltage);
        rightPTOMotors->move_voltage(motorVoltage - straightVoltage);

        pros::delay(20);
    }

    grapher.stopTask();

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

    intakeMotor.move(127);
    driveTo(6);
    intakeMotor.move(0);

    // Back up

    driveTo(-40);

    // Extend back flap, turn to face goal

    vertLeftFlap.set_value(false);
    turnTo(-40);

    // Drive to goal to push 2 acorns in

    driveTo(-12);
    turnTo(-30);
    driveTo(-12);
    turnTo(-20);
    driveTo(-24);

    // Go out of goal, preparing to turn to score intaked acorn

    vertLeftFlap.set_value(true);
    driveTo(7);
    
    // Turn to goal

    turnTo(180);

    // Score acorn, drive out of goal

    driveTo(24);
    driveTo(-16);

    // Turn towards leftmost acorn

    turnTo(-70);
    intakeMotor.move(-127);

    // Drive to leftmost acorn, intake

    intakeMotor.move(127);
    driveTo(48);

    // Turn towards goal, outtake

    intakeMotor.move(0);
    turnTo(35);
    intakeMotor.move(-90);
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

void skillsAuton() {

    // Matchload for 35 seconds

    flywheelMotor.move(127);

    // Drive under pipe

    driveTo(90);

    // Turn to goal

    turnTo(60);

    // Score acorns

    horizLeftFlap.set_value(true);
    horizRightFlap.set_value(true);
    driveTo(12);

    // Back out, turn to face middle field

    driveTo(-12);
    turnTo(-45);

    // Drive to middle field

    driveTo(60);

    // Turn to face goal

    turnTo(90);

    // Score acorns

    horizLeftFlap.set_value(true);
    horizRightFlap.set_value(true);
    driveTo(20);

    // Back out, turn to right side of the field

    horizRightFlap.set_value(false);
    horizLeftFlap.set_value(false);
    driveTo(-20);
    turnTo(-90);

    // Drive to right side of the field

    driveTo(60);

    // Turn to face goal

    turnTo(90);

    // Score acorns

    horizLeftFlap.set_value(true);
    horizRightFlap.set_value(true);
    driveTo(20);
    
    // Back out

    horizRightFlap.set_value(false);
    horizLeftFlap.set_value(false);
    driveTo(-20);

}

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

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */void autonomous() {
    
    //graphy::AsyncGrapher grapher("Drive PID", 20);

    //driveTo(48);
    
    sixBall();

    // skillsAuton();
 }

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors

    static bool isLifted = false;
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