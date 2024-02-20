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

pros::Motor leftTestMotor(11);
pros::Imu imu(10);

pros::Motor intakeMotor(9);
pros::Motor flywheelMotor(7);

pros::adi::DigitalOut horizLeftFlap('A');
pros::adi::DigitalOut horizRightFlap('D');
pros::adi::DigitalOut vertLeftFlap('H');
pros::adi::DigitalOut vertRightFlap('H');

pros::adi::DigitalOut ptoLeft('E');
pros::adi::DigitalOut ptoRight('F');

pros::adi::DigitalOut lockingMech('C');

void turnTo(float degree) {

    lemlib::FAPID turnPID(0, 0, 315, 0, 190, "Turn PID");

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
    drivePID.setExit(2, .1, 500, 300, 3000);
    straightPID.setExit(1, .5, 900, 400, 3000);
    float targetDistance = distance;
    float startDegree = imu.get_rotation();

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
                                        3.14159265 / 180 * 1.375 * .75;

        float motorVoltage = drivePID.update(targetDistance, currentDistanceTraveled);
        float straightVoltage = straightPID.update(startDegree, imu.get_rotation());

        if (motorVoltage > 12000) motorVoltage = 12000;

        grapher.update("Actual Yaw", (imu.get_rotation()) / (startDegree * 2));
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
    //turnTo(-20);
    
    
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

    ptoLeft.set_value(true);
    ptoRight.set_value(true);

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

    //horizLeftFlap.set_value(true);
    //horizRightFlap.set_value(true);
    driveTo(12);



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
    
    //sixBall();

    //threeBall();
    
    fourBallCloseSide();

    //flywheelMotor.move(127);
    
    //closeSideAuton();

    // skillsAuton();
 }

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors

    //leftPTOMotors->set_zero_position_all(0);

    ptoLeft.set_value(true);
    ptoRight.set_value(true);
    
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

    lockingMech.set_value(true);

    while (true) {

        /*=================*/

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !l1Pressed && !l1State) {
            horizLeftFlap.set_value(true);
            horizRightFlap.set_value(true);
            l1Pressed = true;
            l1State = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !l1Pressed && l1State) {
            horizLeftFlap.set_value(false);
            horizRightFlap.set_value(false);
            l1Pressed = true;
            l1State = false;
        } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            l1Pressed = false;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !l2Pressed && !l2State) {
            vertLeftFlap.set_value(true);
            vertRightFlap.set_value(true);
            l2Pressed = true;
            l2State = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !l2Pressed && l2State) {
            vertLeftFlap.set_value(false);
            vertRightFlap.set_value(false);
            l2Pressed = true;
            l2State = false;
        }
        else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            l2Pressed = false;
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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            ptoLeft.set_value(true);
            ptoRight.set_value(true);
            isLifted = false;
        }


        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && isLifted && wentUp) {
            float targetPosition = 360;
            float motorVoltage = liftPID.update(targetPosition, 0);
            leftPTOMotors->move(-motorVoltage);
            rightPTOMotors->move(-motorVoltage);

            if (motorVoltage > 127) {
                motorVoltage = 127;
            }
            else if (motorVoltage < - 127) {
            motorVoltage = -127;
            }

            grapher.update("Actual Yaw", (leftPTOMotors->get_position()) / (targetPosition * 2));
            grapher.update("Target Yaw", (targetPosition / (targetPosition * 2)));
            grapher.update("Voltage", (motorVoltage / 127));

        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && (!wentUp || isLifted)) {
            
            //lockingMech.set_value(true);
            ptoLeft.set_value(false);
            ptoRight.set_value(false);
            isLifted = true;

            leftPTOMotors->move(-30);
            rightPTOMotors->move(-30);

            pros::delay(150);
            
            leftPTOMotors->move(-127);
            rightPTOMotors->move(-127);
            
            float start = pros::millis();

            // get joystick positions
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

            while (pros::millis() < start + 1000) {
                leftBottomMotors->move(leftY);
                rightBottomMotors->move(rightY);

            }

            leftPTOMotors->move(0);
            rightPTOMotors->move(0);

            wentUp = true;

        }         
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && isLifted) {
            leftPTOMotors->move(127);
            rightPTOMotors->move(127);
            wentUp = false;

        }
        else if (!isLifted) {
            leftPTOMotors->move(leftY);
            rightPTOMotors->move(rightY);
        }
        else {
            leftPTOMotors->move(0);
            rightPTOMotors->move(0);
        
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            ptoLeft.set_value(false);
            ptoRight.set_value(false);
            isLifted = true;
        }


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !lockPressed && !lockState) {
            lockingMech.set_value(true);
            lockPressed = true;
            lockState = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !lockPressed && lockState) {
            lockingMech.set_value(false);
            lockPressed = true;
            lockState = false;
        } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            lockPressed = false;
        }

        /*===== DRIVE =====*/

        leftBottomMotors->move(leftY);
        rightBottomMotors->move(rightY);


        pros::delay(20);
    }
}