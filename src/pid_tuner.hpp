#pragma once

// =========================================================================
//  Auto-tuners for LemLib's built-in lateralPID / angularPID
// =========================================================================
//
// HOW THIS WORKS:
//  LemLib exposes `chassis.angularPID` and `chassis.lateralPID` as PUBLIC
//  members specifically so you can swap gains live ("gain scheduling") --
//  see LemLib's chassis.hpp: "Changes are immediate and will affect a
//  motion in progress." We use that hook: before every test we replace the
//  PID object with one built from the kD currently being tested, then call
//  the normal chassis.turnToHeading() / chassis.moveToPoint(). That means
//  every test run uses the EXACT same code path your real auton will use
//  (same slew limiting, same exit conditions you've already configured).
//  While it runs we poll chassis.getPose() ourselves to score overshoot,
//  oscillation, rise time, settle time, and steady-state error -- the same
//  metrics your VEXcode tuner used.
//
//  Note: chassis.angularSettings / chassis.lateralSettings (the exit-
//  condition struct) are PROTECTED in LemLib, so this tool can't touch
//  those, and doesn't need to -- we score settle/overshoot ourselves from
//  raw pose data instead of relying on LemLib's internal exit.
//
// REQUIRES:
//  - A global `lemlib::Chassis chassis;` already constructed (the normal
//    LemLib robot-config setup) and `chassis.calibrate()` already called
//    in initialize().
//  - This file assumes your project's main.h already pulls in pros/api.h
//    and lemlib/api.h (the standard PROS+LemLib template does this).
//
// kI WHILE TUNING: forced to 0 by this tool. kI only acts inside the
// settling band you're measuring, so a nonzero kI adds its own slow
// limit-cycle + phase lag that corrupts the overshoot/oscillation metric
// you're minimizing. Tune kD with kI = 0, then hand-add a small kI
// (~0.005-0.02) afterward purely to erase steady-state error, and re-test
// settling behavior with a plain chassis.turnToHeading()/moveToPoint() call.
//
// HOW TO RUN: both functions BLOCK until the search finishes (~30-90s
// each). Trigger ONCE from a controller button in opcontrol(), e.g.:
//     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
//         angularTunerAuton(2.0);     // tune turning, kP = 2.0
//     }
//     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
//         linearTunerAuton(10.0);     // tune driving straight, kP = 10.0
//     }
// Give the robot clear floor space in every direction -- it WILL turn/
// drive repeatedly and the direction alternates each test so it doesn't
// walk off the table, but give it room regardless.
// Hold B at any time to abort -- the best-so-far kD stays on screen.

struct TuneCfg {
    double kP = 0;               // held constant while kD is searched
    double kDStart = 0;          // bracket start (0 = auto: kP * 2)
    double kDMin = 0;            // bracket floor   (0 = auto: kP * 0.5)
    double kDMax = 0;            // bracket ceiling (0 = auto: kP * 200)
    double growth = 1.4;         // multiply/divide kD by this each bracket step
    int    refineIters = 6;      // golden-section iterations

    double testDuration = 2500;  // ms observed per test (must outlast settling)
    double settleTol = 0;        // "at target" band -- deg for angular, in for linear
    double settleHold = 150;     // ms continuously inside settleTol counts as settled
    double ssWindow = 300;       // ms tail window averaged for steady-state error

    double overshootTol = 0;     // tolerance to call it "oscillation gone"
    int    oscTol = 1;           // error sign-changes allowed

    double wOvershoot = 2.0;     // cost weights -- punish kD too LOW
    double wOscillation = 3.0;
    double wRise = 2.0;          // punish kD too HIGH (early deceleration)
    double wSettle = 1.0;
    double wSteady = 4.0;

    double settleBetween = 400;  // ms pause between tests
};

struct TuneResult {
    double overshoot;
    int    oscillations;
    double riseTime;
    double settleTime;
    double steadyError;
    double cost;
};

// Angular (turning) auto-tuner. Alternates direction each test so the
// robot doesn't walk across the field. testAngleDeg is the size of each
// test turn (90 is a good default -- big enough to see real dynamics,
// small enough to stay in place).
void angularTunerAuton(double kP, double testAngleDeg = 90, double startKD = 0);

// Linear (driving straight) auto-tuner. Alternates forward/backward each
// test along the robot's current heading. testDistanceIn is the length of
// each test leg (24in is a good default for most 12-18" robots).
void linearTunerAuton(double kP, double testDistanceIn = 24, double startKD = 0);