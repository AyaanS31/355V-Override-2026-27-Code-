#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pid_tuner.hpp"

// things to change/check before starting: 

// motors (reversed or not?), drivetrain rpm, drivetrain track width, odom wheel diameter, odom wheels offsets. 
// reverse vertical/horizontal encoders if any error occurs or whatever and test it


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// TODO(wiring): verify reversal. None of ports 1-6 are negated below (a
// negative port number reverses that motor), so currently NO motor in
// either group spins reversed. On most drivetrains the left and right
// motors are mirror-mounted, so one full side typically needs every port
// negated (e.g. {-1, -2, -3}) or your bot will drive in a circle instead
// of straight. Confirm with a bench test (drive both sides forward,
// watch which wheels actually spin forward) before trusting odometry/PID.
pros::MotorGroup leftMotors({-18, -19, -20}, pros::MotorGearset::red); // left motor group - ports 1, 2, 3
pros::MotorGroup rightMotors({15, 16, 17}, pros::MotorGearset::red); // right motor group - ports 4, 5, 6

pros::Motor lift_right(1); 
pros::Motor lift_left(-2);

pros::Motor rotation(11);

pros::adi::Pneumatics clamp('H', true); 
// Inertial Sensor on port 10
pros::Imu imu(21);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// TODO(wiring): verticalEnc is on smart port 11, same physical port as
// `intake` below (pros::Motor intake(11)). Two devices cannot share one
// V5 smart port -- verify on the bench which one is actually plugged into
// port 11 and move the other to a free port (e.g. 14, 15, or 16).
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(9);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.3);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.25, // 11.25 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(7.7, // proportional gain (kP)
                                            0.001, // integral gain (kI)
                                            3.7, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             9, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             20 // maximum acceleration (slew) -- ramps output up instead of snapping to full speed instantly; raise/lower to tune how "abrupt" turns feel
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // set a known origin (0,0,0) so pose readings while driving in opcontrol
    // (see screenTask below) are relative to wherever you physically place
    // the robot at power-on -- that's the point you'll walk it back to when
    // you transcribe coordinates into the auto routines.

    // thread to for brain screen, controller screen, and position logging
    pros::Task screenTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", pose.x); // x
            pros::lcd::print(1, "Y: %f", pose.y); // y
            pros::lcd::print(2, "Theta: %f", pose.theta); // heading
            // TEMP DIAGNOSTIC: raw IMU heading vs LemLib's computed theta above --
            // if these two disagree, the problem is in odom/LemLib config, not the
            // sensor. If they agree (and are both wrong), it's the IMU/wiring itself.
            pros::lcd::print(3, "IMU raw: %f", imu.get_heading());
            // also mirror pose onto the controller screen so it's readable
            // while you're standing next to the robot driving it around to
            // record coordinates for the auto routines. print() is rate
            // limited by VEXos, hence only updating every 50ms below.
            controller.print(0, 0, "X:%.1f Y:%.1f", pose.x, pose.y);
            controller.print(1, 0, "H:%.1f", pose.theta);
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources, also satisfies controller print rate limit
            pros::delay(50);
        }
    });
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



// void move_arm_to_physical_angle(double arm_degrees, int speed) {
//     double motor_degrees = arm_degrees * LIFT_GEAR_RATIO;
//     lift.move_absolute(motor_degrees, speed);
// }

// forward declarations -- needed because autonomous() (below) calls these
// before their bodies appear later in the file
void right_side_auto();
void left_side_auto();
void skills();

void autonomous() {

    right_side_auto();       // red right
    // left_side_auto();     // red left
}


// or probably just take cordinates by driving robot to those places and recording cordinates and coding it.
void right_side_auto() {
    // heading 38.75 = 38.75 degrees CLOCKWISE from facing +Y (LemLib
    // compass convention: 0 = facing +Y, positive = clockwise).
    chassis.setPose(0, 0, 0);


    // NOTE: earlyExitRange only takes effect when minSpeed is non-zero
    // (see MoveToPointParams in chassis.hpp) -- add {.minSpeed = ...} too
    // if you want this movement to actually early-exit at 12in.
    chassis.moveToPoint(0, 8, 600);

    clamp.retract();
    chassis.waitUntilDone();


    chassis.moveToPoint(0, 14, 600);
    chassis.moveToPoint(0, 1, 600, {.forwards = false});
    chassis.turnToHeading(40, 700); 


    chassis.moveToPoint(10, 12, 700, {.maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.turnToHeading(1, 800); 
    chassis.moveToPoint(14, 28, 800, {.maxSpeed = 50});
    pros::delay(700);
    chassis.waitUntilDone();
    clamp.extend();
    pros::delay(200);
    chassis.waitUntilDone();
    chassis.turnToHeading(-4, 400);
    lift_right.move(80);
    lift_left.move(80);
    pros::delay(350);
    lift_right.move(0);
    lift_left.move(0);
    chassis.moveToPoint(11, 3, 800, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(225, 600, {.maxSpeed = 76});

    // // // toggle
    chassis.moveToPoint(1, -7, 600);
    chassis.moveToPoint(11, 3, 800, {.forwards = false});
    chassis.moveToPoint(1, -7, 600); // be in the same point area as the goal
    chassis.moveToPoint(12, 4, 900, {.forwards = false});

    chassis.turnToHeading(315, 600);
    chassis.moveToPoint(29, -10, 800, {.forwards = false, .maxSpeed = 80});
    rotation.move(80);
    pros::delay(1200);
    rotation.move(20);
    pros::delay(620);
    clamp.retract();
    rotation.move(-90);
    pros::delay(900);
    chassis.moveToPoint(26, -7, 700);

    chassis.swingToHeading(225, DriveSide::RIGHT, 1000);
    chassis.moveToPoint(26, -8, 600);
    chassis.swingToHeading(225, DriveSide::RIGHT, 800);
    chassis.moveToPoint(26, -8, 600);
    chassis.swingToHeading(225, DriveSide::RIGHT, 800);
    chassis.moveToPoint(26, -8, 600);
    chassis.swingToHeading(225, DriveSide::RIGHT, 800);
    // chassis.moveToPoint(26, -10.7, 600, {.forwards = false});

    // chassis.moveToPoint(22.5, -14.9, 800, {.maxSpeed = 70});
    // clamp.extend();
    // chassis.waitUntilDone();
    // chassis.moveToPoint(26, -10.7, 800, {.forwards = false});
    // rotation.move(90);
    // pros::delay(600);
    // clamp.extend();

}


void left_side_auto() {
    chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0, 8, 600);

    clamp.retract();
    chassis.waitUntilDone();

    chassis.moveToPoint(0, 14, 600);
    chassis.moveToPoint(0, 1, 600, {.forwards = false});
    chassis.turnToHeading(-40, 700); 


    // chassis.moveToPoint(10, 12, 700, {.maxSpeed = 80});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(1, 800); 
    // chassis.moveToPoint(14, 28, 800, {.maxSpeed = 50});
    // pros::delay(700);
    // chassis.waitUntilDone();
    // clamp.extend();
    // pros::delay(200);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-4, 400);
    // lift_right.move(80);
    // lift_left.move(80);
    // pros::delay(350);
    // lift_right.move(0);
    // lift_left.move(0);
    // chassis.moveToPoint(11, 3, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(225, 600, {.maxSpeed = 76});

    // // // // toggle
    // chassis.moveToPoint(1, -7, 600);
    // chassis.moveToPoint(11, 3, 800, {.forwards = false});
    // chassis.moveToPoint(1, -7, 600); // be in the same point area as the goal
    // chassis.moveToPoint(12, 4, 900, {.forwards = false});

    // chassis.turnToHeading(315, 600);
    // chassis.moveToPoint(28, -10.5, 800, {.forwards = false, .maxSpeed = 80});
    // rotation.move(80);
    // pros::delay(1600);
    // clamp.retract();
    // rotation.move(-90);
    // pros::delay(900);
    // chassis.cancelMotion();
    // chassis.moveToPoint(26, -7, 700);

    // chassis.swingToHeading(300, DriveSide::RIGHT, 1300);

    // chassis.moveToPoint(22.5, -14.9, 800, {.maxSpeed = 70});
    // clamp.extend();
    // chassis.waitUntilDone();
    // chassis.moveToPoint(26, -10.7, 800, {.forwards = false});
    // rotation.move(90);
    // pros::delay(600);
    // clamp.extend();
}

void skills(){

}
/**
 * Runs in driver control
 */

bool is_clamp_flipped = true;
bool clampOpen = true;

// how long the lift runs for after a single L1 tap, and at what speed --
// tune LIFT_BUMP_MS until it lifts "a little bit" the way you want

void opcontrol(){

    while (true) {
        int left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(left_y, right_y);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift_right.move(127);  // Lift goes up, only for LIFT_BUMP_MS
            lift_left.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift_right.move(-127); // Lift goes down
            lift_left.move(-127);
        } else {
            lift_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            lift_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            lift_right.move(0);      // Lock the lift in place
            lift_left.move(0);
        }

        // the upper rotation if needed code 
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            rotation.move(90);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            rotation.move(-90);
        } else{
            rotation.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            rotation.move(0); 
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            clampOpen = !clampOpen;
            if (clampOpen) {
                clamp.extend();
            } else {
                clamp.retract();
            }
        };

        pros::delay(20);


    };
};