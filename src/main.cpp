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

// left motors on ports 8, 20, and 19. Motors on ports 8 and 20 are reversed. Using blue gearbox
auto leftBottomMotors = lemlib::makeMotorGroup({-20, 19, 16, 17}, pros::v5::MotorGears::blue);
// right motors on ports 2, 11, and 13. Motor on port 13 is reversed. Using blue gearbox
auto rightBottomMotors = lemlib::makeMotorGroup({-10, -18, -9, 6}, pros::v5::MotorGears::blue);

pros::adi::DigitalOut intake('F');
pros::adi::DigitalOut wings('D');

//pros::Motor leftTestMotor(11);
pros::Imu imu(2);
pros::GPS gps(12);

void turnTo(float degree) {
    lemlib::FAPID turnPID(0, 0, 200, 0, 532, "Turn PID");

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    turnPID.setExit(1, .5, 800, 500, 1250);

    float targetDistance = degree + imu.get_rotation();

    while (!turnPID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage
        int motorVoltage = turnPID.update(targetDistance, imu.get_rotation());

        leftBottomMotors->move_voltage(motorVoltage);
        rightBottomMotors->move_voltage(-motorVoltage);
        //leftPTOMotors->move_voltage(motorVoltage);
        //rightPTOMotors->move_voltage(-motorVoltage);

        pros::delay(20);
    }
}

void driveTo(float distance) {
    graphy::AsyncGrapher grapher("Drive PID", 20);

    lemlib::FAPID drivePID(0, 0, 1000, 0, 2800, "Drive PID"); // 2000
    lemlib::FAPID straightPID(0, 0, 400, 0, 100, "Turn PID");

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    drivePID.setExit(2, .1, 500, 300, 3000);
    straightPID.setExit(1, .5, 900, 400, 3000);
    float targetDistance = distance;
    float startDegree = imu.get_rotation();

    leftBottomMotors->set_zero_position_all(0);
    rightBottomMotors->set_zero_position_all(0);
    //leftPTOMotors->set_zero_position_all(0);
    //rightPTOMotors->set_zero_position_all(0);

    grapher.addDataType("Actual Yaw", pros::c::COLOR_CYAN);
    grapher.addDataType("Target Yaw", pros::c::COLOR_RED);
    grapher.addDataType("Voltage", pros::c::COLOR_YELLOW);

    grapher.startTask();

    while (!drivePID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage

        float currentDistanceTraveled = ((leftBottomMotors->get_position(0) + rightBottomMotors->get_position()) / 2) *
                                        3.14159265 / 180 * 1.375 * .75;

        float motorVoltage = drivePID.update(targetDistance, currentDistanceTraveled);
        float straightVoltage = straightPID.update(startDegree, imu.get_rotation());

        if (motorVoltage > 12000) motorVoltage = 12000;

        grapher.update("Actual Yaw", (imu.get_rotation()) / (startDegree * 2));
        grapher.update("Target Yaw", (startDegree / (startDegree * 2)));
        grapher.update("Voltage", (straightVoltage / 12000));

        leftBottomMotors->move_voltage(motorVoltage + straightVoltage);
        rightBottomMotors->move_voltage(motorVoltage - straightVoltage);
        //leftPTOMotors->move_voltage(motorVoltage + straightVoltage);
        //rightPTOMotors->move_voltage(motorVoltage - straightVoltage);

        pros::delay(20);
    }

    grapher.stopTask();
}

void circleArcTo(float radius, float finalTheta) {
    graphy::AsyncGrapher grapher("Drive PID", 20);

    lemlib::FAPID drivePID(0, 0, 1000, 0, 4750, "Drive PID"); // 2000

    // Large error and small error are the ranges where the loop can exit. Small is the important one.
    // Large and small times are for how long the bot must be within the range to exit. Max Time is
    // the time it takes for the bot to give up on the PID loop and exit.
    drivePID.setExit(2, .1, 500, 300, 6000);

    //float startDegree = imu.get_rotation() / 180 * 3.14159265;
    float targetDistanceLeft, targetDistanceRight;
    float trackWidth = 10.125;

    if (finalTheta > 0) {
        targetDistanceLeft = radius * (finalTheta);
        targetDistanceRight = (radius + trackWidth) * (finalTheta);
    } else {
        targetDistanceRight = radius * (finalTheta);
        targetDistanceLeft = (radius + trackWidth) * (finalTheta);
    }

    leftBottomMotors->set_zero_position_all(0);
    rightBottomMotors->set_zero_position_all(0);
    //leftPTOMotors->set_zero_position_all(0);
    //rightPTOMotors->set_zero_position_all(0);

    grapher.addDataType("Left Dist", pros::c::COLOR_CYAN);
    grapher.addDataType("Right Dist", pros::c::COLOR_RED);
    grapher.addDataType("Voltage", pros::c::COLOR_YELLOW);

    grapher.startTask();

    while (!drivePID.settled()) { // While the bot is still moving / oscillating

        // Update the PID loop and get the motor voltage

        float currentDistanceTraveledLeft = leftBottomMotors->get_position(0) * 3.14159265 / 180 * 1.375 * .75;

        float currentDistanceTraveledRight = rightBottomMotors->get_position(0) * 3.14159265 / 180 * 1.375 * .75;

        float leftMotorVoltage, rightMotorVoltage;
        if (finalTheta < 0) {
            leftMotorVoltage = drivePID.update((radius + trackWidth) * (finalTheta), currentDistanceTraveledLeft);
            rightMotorVoltage =
                drivePID.update((radius + trackWidth) * (finalTheta) / (1 + (trackWidth / radius)), currentDistanceTraveledRight);

        } else {
            leftMotorVoltage =
                drivePID.update((radius + trackWidth) * (finalTheta) / (1 + (trackWidth / radius)), currentDistanceTraveledRight);
            rightMotorVoltage = drivePID.update((radius + trackWidth) * (finalTheta), currentDistanceTraveledLeft);
        }

        if (leftMotorVoltage > 12000) leftMotorVoltage = 12000;
        if (rightMotorVoltage > 12000) rightMotorVoltage = 12000;
        if (leftMotorVoltage < -12000) leftMotorVoltage = -12000;
        if (rightMotorVoltage < -12000) rightMotorVoltage = -12000;
        
        grapher.update("Left Dist", (currentDistanceTraveledLeft) / (radius + trackWidth));
        grapher.update("Right Dist", (currentDistanceTraveledRight / (radius + trackWidth)));
        grapher.update("Voltage", (rightMotorVoltage / 12000));

        if (finalTheta > 0) {
            rightBottomMotors->move_voltage(rightMotorVoltage);
            //rightPTOMotors->move_voltage(rightMotorVoltage);
        } else {
            leftBottomMotors->move_voltage(leftMotorVoltage);
            //leftPTOMotors->move_voltage(leftMotorVoltage);
        }
        // rightBottomMotors->move_voltage(rightMotorVoltage);
        // rightPTOMotors->move_voltage(rightMotorVoltage);

        pros::delay(20);
    }

    grapher.stopTask();
}

/*
void threeBall() {
    ptoLeft.set_value(true);
    ptoRight.set_value(true);

    // Intake acorn under bar

    intakeMotor.move(127);
    driveTo(6);
    pros::delay(500);

    // Back up

    driveTo(-40);

    // Extend back flap, turn to face goal

    vertRightFlap.set_value(true);
    vertRightFlap.set_value(true);
    turnTo(-40);

    // Drive to goal to push 2 acorns in

    driveTo(-14);
    turnTo(-35);
    intakeMotor.move(0);
    driveTo(-16);
    vertLeftFlap.set_value(false);
    vertRightFlap.set_value(false);
    // turnTo(-20);

    leftBottomMotors->move_voltage(-12000);
    rightBottomMotors->move_voltage(-12000);
    leftPTOMotors->move_voltage(-12000);
    rightPTOMotors->move_voltage(-12000);

    pros::delay(2000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    // Go out of goal, preparing to turn to score intaked acorn

    driveTo(16);

    // Turn to goal

    turnTo(-160);
    intakeMotor.move(-127);

    // Score acorn, drive out of goal repeadetly

    leftBottomMotors->move_voltage(12000);
    rightBottomMotors->move_voltage(12000);
    leftPTOMotors->move_voltage(12000);
    rightPTOMotors->move_voltage(12000);

    pros::delay(2000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    driveTo(-16);

    leftBottomMotors->move_voltage(12000);
    rightBottomMotors->move_voltage(12000);
    leftPTOMotors->move_voltage(12000);
    rightPTOMotors->move_voltage(12000);

    pros::delay(2000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    driveTo(-16);

    leftBottomMotors->move_voltage(12000);
    rightBottomMotors->move_voltage(12000);
    leftPTOMotors->move_voltage(12000);
    rightPTOMotors->move_voltage(12000);

    pros::delay(2000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    driveTo(-16);
}

void sixBall() {
    //ptoLeft.set_value(true);
    //ptoRight.set_value(true);

    // Intake acorn under bar

    intakeMotor.move(65);
    driveTo(6);
    pros::delay(500);
    intakeMotor.move(0);

    // Back up

    driveTo(-40);

    // Extend back flap, turn to face goal

    vertRightFlap.set_value(true);
    vertRightFlap.set_value(true);
    turnTo(-40);

    // Drive to goal to push 2 acorns in

    driveTo(-14);
    turnTo(-25);
    vertLeftFlap.set_value(false);
    vertRightFlap.set_value(false);
    driveTo(-16);
    turnTo(-10);

    leftBottomMotors->move_voltage(-12000);
    rightBottomMotors->move_voltage(-12000);
    leftPTOMotors->move_voltage(-12000);
    rightPTOMotors->move_voltage(-12000);

    pros::delay(2000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    // Go out of goal, preparing to turn to score intaked acorn

    driveTo(24);

    // Turn to goal

    turnTo(-150);

    // Score acorn, drive out of goal

    leftBottomMotors->move_voltage(12000);
    rightBottomMotors->move_voltage(12000);
    leftPTOMotors->move_voltage(12000);
    rightPTOMotors->move_voltage(12000);

    pros::delay(2000);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    driveTo(-16);

    // Turn towards leftmost acorn

    turnTo(-70);
    intakeMotor.move(-127);

    // Drive to leftmost acorn, intake

    intakeMotor.move(127);
    driveTo(48);

    // Turn towards goal, outtake

    intakeMotor.move(0);
    turnTo(135);
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

void fiveBallFarSide() {

    ptoLeft.set_value(false);
    ptoRight.set_value(false);

    // Rush
    intakeMotor.move(127);
    //horizLeftFlap.set_value(true);
    //horizRightFlap.set_value(true);
    driveTo(56);

    // Drive Back to put by the goal

    driveTo(-8);

    turnTo(90);
    intakeMotor.move(-127);
    pros::delay(500);
    turnTo(170);
    intakeMotor.move(127);

    // Get corner acorn

    driveTo(24);
    driveTo(-12);

    // Turn to goal, score

    turnTo(-180);

    horizLeftFlap.set_value(true);
    horizRightFlap.set_value(true);

    driveTo(25);

    // Go back, turn to matchload bar

    driveTo(-12);
    turnTo(90);

    // Drive to matchload bar

    driveTo(20);
    turnTo(90);

    // Descore, score


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

void closeSideAuton() {
    ptoLeft.set_value(true);
    ptoRight.set_value(true);

    vertRightFlap.set_value(true);
    vertRightFlap.set_value(true);

    pros::delay(500);

    turnTo(-90);

    vertRightFlap.set_value(false);
    vertRightFlap.set_value(false);

    turnTo(45);

    driveTo(-16);

    turnTo(-50);

    driveTo(-30);
}

void fourBallCloseSide() {
    ptoLeft.set_value(true);
    ptoRight.set_value(true);

    vertRightFlap.set_value(true);
    vertRightFlap.set_value(true);

    pros::delay(500);

    turnTo(-90);

    vertRightFlap.set_value(false);
    vertRightFlap.set_value(false);

    turnTo(115);

    // Mid Ball Rush

    intakeMotor.move(127);
    driveTo(48);
    driveTo(-2);

    // Turn to other side of field

    turnTo(70);

    // Push balls over barrier

    horizLeftFlap.set_value(true);
    horizRightFlap.set_value(true);
    intakeMotor.move(127);

    leftBottomMotors->move_voltage(12000);
    rightBottomMotors->move_voltage(12000);
    leftPTOMotors->move_voltage(12000);
    rightPTOMotors->move_voltage(12000);

    pros::delay(500);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    // Turn back to matchload bar

    horizLeftFlap.set_value(false);
    horizRightFlap.set_value(false);
    driveTo(-12);
    turnTo(130);

    // Go to matchload bar

    driveTo(48);

    // Regular close side

    turnTo(-100);

    intakeMotor.move(-127);
    driveTo(24);

    turnTo(-20);

    // horizLeftFlap.set_value(true);
    // horizRightFlap.set_value(true);
    driveTo(12);
}

void midBallRushCloseSide() {

    ptoLeft.set_value(false);
    ptoRight.set_value(false);

    // Rush
    intakeMotor.move(127);
    driveTo(52);

    // Turn to barrier
    turnTo(70);

    horizLeftFlap.set_value(true);
    horizRightFlap.set_value(true);

    // Push over
    leftBottomMotors->move_voltage(7000);
    rightBottomMotors->move_voltage(6000);
    leftPTOMotors->move_voltage(7000);
    rightPTOMotors->move_voltage(6000);

    pros::delay(750);

    leftBottomMotors->move_voltage(0);
    rightBottomMotors->move_voltage(0);
    leftPTOMotors->move_voltage(0);
    rightPTOMotors->move_voltage(0);

    horizLeftFlap.set_value(false);
    horizRightFlap.set_value(false);

    // Drive back, turn to matchload bar

    driveTo(-12);

    turnTo(-55);

    // Go to matchload bar

    driveTo(-56);

    // Descore

    vertRightFlap.set_value(true);

    //circleArcTo(12, -100);

    turnTo(-100);

    pros::delay(500);

    vertRightFlap.set_value(false);

    turnTo(180);

    // Drive under matchload bar
    
    //horizLeftFlap.set_value(true);
    //horizRightFlap.set_value(true);
    intakeMotor.move(-127);
    driveTo(42);


    
} */

void mavA() {

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
    gps.initialize_full(10, 10, 10, 0, 0);
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
void autonomous() {

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors

    // leftPTOMotors->set_zero_position_all(0);

    //ptoLeft.set_value(false);
    //ptoRight.set_value(false);

    graphy::AsyncGrapher grapher("Lift PID", 20);

    grapher.addDataType("Actual Yaw", pros::c::COLOR_CYAN);
    grapher.addDataType("Target Yaw", pros::c::COLOR_RED);
    grapher.addDataType("Voltage", pros::c::COLOR_YELLOW);

    static bool isLifted = false;
    static bool wentUp = false;

    static bool l1Pressed = false;
    static bool l1State = false;

    static bool l2Pressed = false;
    static bool l2State = false;

    static bool lockPressed = false;
    static bool lockState = false;

    lemlib::FAPID liftPID(.0175, 0, 0, 0, 0, "Drive PID"); // 2000

    grapher.startTask();

    //lockingMech.set_value(true);

    while (true) {
        /*=================*/

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        /*===== DRIVE =====*/

        leftBottomMotors->move(leftY);
        rightBottomMotors->move(rightY);

        pros::delay(20);
    }
}