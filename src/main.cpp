#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

enum AutonSelect : int {
    lemlibA = 420,
    customA = 6969,
};

int AUTON_SELECT = customA;

// forward-declaring auton funcs
void customForward();
void lemlibForward();

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({
                            19,
                            20
                            }, pros::MotorGearset::green);

pros::MotorGroup rightMotors({
                            16,
                            17
                            }, pros::MotorGearset::green);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 12 inch track width
                              lemlib::Omniwheel::OLD_4, // WHEEL TYPE
                              200, // drivetrain rpm is 200
                              5 // horizontal drift: 2 for all-omni tank drive, 8 for omni-traction tank drive
);

// vertical motion controller
lemlib::ControllerSettings linearController(2, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            8, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3,    // joystick deadband out of 127
                                  10,   // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors
    if (!pros::lcd::is_initialized()) pros::lcd::initialize();
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(0,0,0);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

    switch (AUTON_SELECT) {
    case lemlibA:
        lemlibForward();
        break;
    case customA:
        customForward();
        break;
    }

}

void opcontrol() {
    while (true) {
		double forward = controller.get_analog(ANALOG_LEFT_Y);
	  	double angular = controller.get_analog(ANALOG_RIGHT_X);
		leftMotors.move(forward+angular);
        rightMotors.move(-forward-angular);

		pros::delay(20);                             
	}
}


//------------------------------------------ AUTONS ------------------------------------------------------//
// PATHS

void lemlibForward() {
    chassis.moveToPose(0, 10, 0, 10000);
}

void customForward() {
    leftMotors.move(70);
    rightMotors.move(70);
    pros::delay(1000);
    leftMotors.move(0);
    rightMotors.move(0);
}
