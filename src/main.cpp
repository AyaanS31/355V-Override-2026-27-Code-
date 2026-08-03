#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>

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
pros::MotorGroup leftMotors({1, 2, 3}, pros::MotorGearset::red); // left motor group - ports 1, 2, 3
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::red); // right motor group - ports 4, 5, 6

pros::MotorGroup lift_right({13}); 
pros::MotorGroup lift_left({12});

pros::Motor intake(11);

pros::adi::Pneumatics claw('A', false); 
pros::adi::Pneumatics claw_rotate('B', false); 
// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// TODO(wiring): verticalEnc is on smart port 11, same physical port as
// `intake` below (pros::Motor intake(11)). Two devices cannot share one
// V5 smart port -- verify on the bench which one is actually plugged into
// port 11 and move the other to a free port (e.g. 14, 15, or 16).
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
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
                                             0, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
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

// ============================================================================
//  Blue-alliance coordinate flip
// ============================================================================
// Override's field is 180-degree POINT symmetric between alliances -- the
// red and blue starting tiles are diagonally opposite through the exact
// center of the field, not mirrored across a single axis. So every red-side
// coordinate maps to blue with a full point reflection through the origin:
// (x, y) -> (-x, -y), and heading rotates a full 180 degrees.
//
// VERIFY THIS against your actual field before trusting it: look at the
// field from above. If red-right's starting tile, rotated 180 degrees about
// the center, lands you on blue's tile, this (x,y)->(-x,-y) reflection is
// correct. If instead the alliance tiles are mirrored across just one wall
// (a plain left/right or up/down flip, not a rotation), you'd want
// (x, -y) or (-x, y) instead -- that's a different field layout and this
// function would need to change.
float flipX(float x, bool isBlue) { return isBlue ? -x : x; }
float flipY(float y, bool isBlue) { return isBlue ? -y : y; }
float flipHeading(float thetaDeg, bool isBlue) {
    if (!isBlue) return thetaDeg;
    float h = std::fmod(thetaDeg + 180.0f, 360.0f);
    if (h < 0) h += 360.0f;
    return h;
}


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
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
void right_side_auto(bool isBlue = false);
void left_side_auto(bool isBlue = false);
void skills();

// TODO(strategy): pick which of these 4 actually runs, e.g. with an LCD
// selector read in initialize(). right_side_auto(true) is red-right's
// route flipped 180 degrees -- confirm on the real field which blue
// quadrant that geometrically lands you in (see flipX/flipY comment above)
// before assuming "right" means the same physical side for both alliances.
void autonomous() {
    right_side_auto();       // red right
    // left_side_auto();     // red left
    // right_side_auto(true); // blue (flipped from red right)
    // left_side_auto(true);  // blue (flipped from red left)
}


// or probably just take cordinates by driving robot to those places and recording cordinates and coding it. 
void right_side_auto(bool isBlue) {
    // TODO(wiring): confirm this heading matches how the robot is
    // physically placed on the tile (0 = facing +Y, LemLib's compass
    // convention, increasing clockwise).
    chassis.setPose(flipX(-11.443, isBlue), flipY(-64.429, isBlue), flipHeading(0, isBlue));

    chassis.turnToHeading(flipHeading(-30, isBlue), 200);
    chassis.waitUntilDone();

    // NOTE: earlyExitRange only takes effect when minSpeed is non-zero
    // (see MoveToPointParams in chassis.hpp) -- add {.minSpeed = ...} too
    // if you want this movement to actually early-exit at 12in.
    chassis.moveToPoint(flipX(-18.159, isBlue), flipY(-53.483, isBlue), 1000, {.earlyExitRange = 12});
    // lift and score --> See the mechanism of how it works in the robot and program the scoring accordingly.
    // fixed: `lift` doesn't exist, your two lift motor groups are lift_left/lift_right
    lift_left.move(127);
    lift_right.move(127);
    claw.retract();
    chassis.waitUntilDone();


    chassis.moveToPoint(flipX(-16.667, isBlue), flipY(-27.612, isBlue), 1400);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-19.154, isBlue), flipY(-26.120, isBlue), 800);
    chassis.waitUntilDone();
    // TODO(strategy): mechanism action here? path reverses out of this
    // corner immediately after -- likely an intake/clamp spot.

    chassis.moveToPoint(flipX(-1.493, isBlue), flipY(-44.528, isBlue), 1400, {.forwards = false});
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-1.244, isBlue), flipY(-64.926, isBlue), 1200);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(18.159, isBlue), flipY(-50.498, isBlue), 1400);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(0.995, isBlue), flipY(-66.419, isBlue), 1300);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-41.294, isBlue), flipY(-50.001, isBlue), 2000);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-32.090, isBlue), flipY(-48.508, isBlue), 900);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-23.383, isBlue), flipY(-65.424, isBlue), 1200);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-23.135, isBlue), flipY(-53.981, isBlue), 900);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(22.140, isBlue), flipY(-65.175, isBlue), 2100);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(22.637, isBlue), flipY(-53.981, isBlue), 900);
    chassis.waitUntilDone();
}


void left_side_auto(bool isBlue) {
    // TODO(wiring): confirm this heading matches how the robot is
    // physically placed on the tile.
    chassis.setPose(flipX(-66.057, isBlue), flipY(7.823, isBlue), flipHeading(0, isBlue));

    chassis.moveToPoint(flipX(-51.902, isBlue), flipY(-16.514, isBlue), 1400);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-30.297, isBlue), flipY(-22.226, isBlue), 1300);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-44.700, isBlue), flipY(-1.118, isBlue), 1400, {.forwards = false});
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-64.319, isBlue), flipY(-0.869, isBlue), 1200);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-49.170, isBlue), flipY(19.246, isBlue), 1400);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-62.580, isBlue), flipY(21.978, isBlue), 1000);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-52.647, isBlue), flipY(22.971, isBlue), 900, {.forwards = false});
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-66.305, isBlue), flipY(-19.246, isBlue), 2000);
    chassis.waitUntilDone();

    chassis.moveToPoint(flipX(-53.143, isBlue), flipY(-22.226, isBlue), 1000);
    chassis.waitUntilDone();
}

void skills(){

}
/**
 * Runs in driver control
 */

bool is_claw_flipped = false;
bool clawOpen = false; 

void opcontrol() {
    while (true) {
        int left_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_y = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(left_y, right_y);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift_right.move(127);  // Lift goes up
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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move(-127);
        }  else {
            intake.move(0);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            clawOpen = !clawOpen;
            if (clawOpen) {
                claw.extend();
            } else {
                claw.retract();
            }
        }

        // claw rotation toggle
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            is_claw_flipped = !is_claw_flipped;
            if (is_claw_flipped) {
                claw_rotate.extend();
            } else {
                claw_rotate.retract();
            }
        }
        pros::delay(20); 
        // only uncomment once start tuning PID 

        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        //     angularTunerAuton(2.0);     // tune turning, kP = 2.0
        // }
        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        //     linearTunerAuton(10.0);     // tune driving straight, kP = 10.0
        // }
    }
}