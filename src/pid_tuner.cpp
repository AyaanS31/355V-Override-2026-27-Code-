#include "pid_tuner.hpp"
#include "main.h"   // pulls in pros/api.h + lemlib/api.h + your `chassis` declaration
#include <cmath>
#include <functional>
#include <new>

// Use your own controller object instead if you already have one declared
// globally -- having two pros::Controller objects bound to the same
// physical controller is fine, but if you'd rather not, just delete this
// line and pass your existing controller into the functions below (or
// `extern pros::Controller master;` it from your robot-config).
static pros::Controller tunerMaster(pros::E_CONTROLLER_MASTER);

constexpr double kPi = 3.14159265358979323846;

static bool g_tuneAborted = false;
static bool g_stopTuning = false;

static int sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// Direct physical kill. The tuner blocks opcontrol()/autonomous(), so a
// flag you set from those tasks can never be checked -- holding B here
// flips it from inside the tuner's own polling loop instead.
static bool tunerKilled() {
    if (tunerMaster.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        g_stopTuning = true;
    }
    return g_stopTuning;
}

static double computeCost(const TuneCfg& cfg, const TuneResult& r) {
    return cfg.wOvershoot * r.overshoot + cfg.wOscillation * (double)r.oscillations +
           cfg.wRise * (r.riseTime / 1000.0) + cfg.wSettle * (r.settleTime / 1000.0) +
           cfg.wSteady * r.steadyError;
}

static void printTuneStatus(const char* phase, double kP, double kD, const TuneResult& r, double bestKD,
                             double bestCost) {
    pros::lcd::print(0, "PID Tuner [%s]", phase);
    pros::lcd::print(1, "kP %.3f  testing kD %.3f", kP, kD);
    pros::lcd::print(2, "overshoot %.2f", r.overshoot);
    pros::lcd::print(3, "oscillations %d", r.oscillations);
    pros::lcd::print(4, "rise %.0f  settle %.0f ms", r.riseTime, r.settleTime);
    pros::lcd::print(5, "steady err %.2f", r.steadyError);
    pros::lcd::print(6, "cost %.2f  best kD %.3f (%.2f)", r.cost, bestKD, bestCost);

    tunerMaster.clear();
    tunerMaster.print(0, 0, "kD %.2f c%.1f", kD, r.cost);
    tunerMaster.print(1, 0, "best %.3f", bestKD);
}

// Runs one test at the given kD via testFn, scores it, tracks the best
// result so far, prints status, pauses, and returns the result.
static TuneResult runTestAt(const TuneCfg& cfg, double kD, const char* phase,
                             const std::function<TuneResult(double)>& testFn, double& bestKD, double& bestCost) {
    TuneResult r = testFn(kD);
    r.cost = computeCost(cfg, r);

    if (r.cost < bestCost) {
        bestCost = r.cost;
        bestKD = kD;
    }
    printTuneStatus(phase, cfg.kP, kD, r, bestKD, bestCost);
    if (!g_tuneAborted) pros::delay((uint32_t)cfg.settleBetween);
    return r;
}

// Generic bracket + golden-section search, shared by both tuners.
// Phase 1 brackets the optimum: we want kDLow to OSCILLATE and kDHigh to
// be STABLE, so the refine search sits between them.
// Phase 2 golden-section searches that bracket to minimize cost.
static void runSearch(const TuneCfg& cfg, const std::function<TuneResult(double)>& testFn, double& bestKD,
                       double& bestCost) {
    double kDStart = cfg.kDStart > 0 ? cfg.kDStart : cfg.kP * 2.0;
    if (kDStart <= 0) kDStart = 1.0;
    double kDMax = cfg.kDMax > 0 ? cfg.kDMax : cfg.kP * 200.0;
    if (kDMax <= 0) kDMax = 200.0;
    double kDMin = cfg.kDMin > 0 ? cfg.kDMin : cfg.kP * 0.5;
    if (kDMin <= 0) kDMin = 0.1;

    double kDLow = kDMin;
    double kDHigh = kDMax;

    TuneResult r0 = runTestAt(cfg, kDStart, "BRACKET", testFn, bestKD, bestCost);
    bool startStable = (r0.overshoot <= cfg.overshootTol) && (r0.oscillations <= cfg.oscTol);

    if (startStable) {
        // kDStart already kills oscillation -- it may be TOO high (decelerating
        // early). Walk DOWN until oscillation reappears, so we find the
        // smallest stable kD (the fastest one).
        kDHigh = kDStart;
        double kD = kDStart / cfg.growth;
        while (kD >= kDMin) {
            if (g_tuneAborted) break;
            TuneResult r = runTestAt(cfg, kD, "DESCEND", testFn, bestKD, bestCost);
            bool stable = (r.overshoot <= cfg.overshootTol) && (r.oscillations <= cfg.oscTol);
            if (!stable) {
                kDLow = kD;
                break;
            }
            kDHigh = kD;
            kD = kD / cfg.growth;
        }
    } else {
        // kDStart oscillates -- climb UP until oscillation is eliminated.
        kDLow = kDStart;
        bool found = false;
        double kD = kDStart * cfg.growth;
        while (kD <= kDMax) {
            if (g_tuneAborted) break;
            TuneResult r = runTestAt(cfg, kD, "ASCEND", testFn, bestKD, bestCost);
            bool stable = (r.overshoot <= cfg.overshootTol) && (r.oscillations <= cfg.oscTol);
            if (stable) {
                kDHigh = kD;
                found = true;
                break;
            }
            kDLow = kD;
            kD = kD * cfg.growth;
        }
        if (!found) kDHigh = kDMax;
    }

    double a = kDLow;
    double b = kDHigh;
    const double invphi = 0.6180339887;
    double c = b - invphi * (b - a);
    double d = a + invphi * (b - a);
    double fc = 0;
    double fd = 0;
    if (!g_tuneAborted) fc = runTestAt(cfg, c, "REFINE", testFn, bestKD, bestCost).cost;
    if (!g_tuneAborted) fd = runTestAt(cfg, d, "REFINE", testFn, bestKD, bestCost).cost;

    for (int i = 0; i < cfg.refineIters; i++) {
        if (g_tuneAborted) break;
        if (fc < fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - invphi * (b - a);
            fc = runTestAt(cfg, c, "REFINE", testFn, bestKD, bestCost).cost;
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + invphi * (b - a);
            fd = runTestAt(cfg, d, "REFINE", testFn, bestKD, bestCost).cost;
        }
    }
}

static void printFinal(const char* label, double kP, double bestKD, double bestCost) {
    pros::lcd::print(0, g_tuneAborted ? "TUNING ABORTED - best so far" : "TUNING COMPLETE");
    pros::lcd::print(1, "%s", label);
    pros::lcd::print(2, "kP   %.3f", kP);
    pros::lcd::print(3, "kD   %.3f  <-- use this", bestKD);
    pros::lcd::print(4, "cost %.2f", bestCost);

    tunerMaster.clear();
    tunerMaster.print(0, 0, "%s kP%.2f", label, kP);
    tunerMaster.print(1, 0, "best kD %.3f", bestKD);
    tunerMaster.rumble("---");
}

// =========================================================================
//  ANGULAR (turning) tuner
// =========================================================================

void angularTunerAuton(double kP, double testAngleDeg, double startKD) {
    pros::lcd::initialize();
    g_tuneAborted = false;
    g_stopTuning = false;

    TuneCfg cfg;
    cfg.kP = kP;
    cfg.kDStart = startKD;
    cfg.settleTol = 0.75;    // deg
    cfg.overshootTol = 1.5;  // deg

    double dir = 1.0;

    auto testFn = [&](double kD) -> TuneResult {
        TuneResult r{};
        r.overshoot = 0;
        r.oscillations = 0;
        r.riseTime = cfg.testDuration;
        r.settleTime = cfg.testDuration;
        r.steadyError = 0;

        // Gain-schedule swap -- takes effect immediately, see chassis.hpp.
        // kI forced to 0 while bracketing/refining kD (see header notes).
        // NOTE: lemlib::PID stores its gains as `const` members, so it has no
        // usable copy-assignment operator -- `chassis.angularPID = PID(...)`
        // will not compile. Placement-new reconstructs the object in place
        // instead (safe here: PID is trivially destructible, just floats/bool).
        new (&chassis.angularPID) lemlib::PID(cfg.kP, 0, kD, 0, false);

        lemlib::Pose start = chassis.getPose();
        double target = start.theta + dir * testAngleDeg;
        dir = -dir;

        // async = true: turnToHeading() spins its own task and returns
        // immediately so we can poll the pose ourselves below, exactly
        // like the original tuner's tight measurement loop.
        chassis.turnToHeading(target, (int)cfg.testDuration + 500, {}, true);

        uint32_t startTime = pros::millis();
        uint32_t elapsed = 0;
        int initialSign = 0;
        int lastSign = 0;
        bool reached = false;
        bool settled = false;
        double settleEnterTime = -1;
        double ssSum = 0;
        int ssCount = 0;

        while (elapsed < (uint32_t)cfg.testDuration) {
            if (tunerKilled()) {
                g_tuneAborted = true;
                break;
            }

            double error = lemlib::angleError(target, chassis.getPose().theta, false);
            int s = sign(error);

            if (initialSign == 0 && s != 0) {
                initialSign = s;
                lastSign = s;
            }
            if (s != 0 && lastSign != 0 && s != lastSign) r.oscillations++;
            if (s != 0) lastSign = s;

            // overshoot = error on the opposite side of where it started
            if (initialSign != 0 && s == -initialSign) {
                double mag = fabs(error);
                if (mag > r.overshoot) r.overshoot = mag;
            }

            if (!reached && fabs(error) <= cfg.settleTol) {
                reached = true;
                r.riseTime = elapsed;
            }

            if (fabs(error) <= cfg.settleTol) {
                if (settleEnterTime < 0) settleEnterTime = elapsed;
                if (!settled && (elapsed - settleEnterTime) >= cfg.settleHold) {
                    settled = true;
                    r.settleTime = settleEnterTime;
                }
            } else {
                settleEnterTime = -1;
            }

            if (elapsed >= cfg.testDuration - cfg.ssWindow) {
                ssSum += fabs(error);
                ssCount++;
            }

            pros::delay(10);
            elapsed = pros::millis() - startTime;
        }

        chassis.cancelMotion();  // make sure nothing's still driving before the next test
        if (ssCount > 0) r.steadyError = ssSum / ssCount;
        return r;
    };

    double bestKD = cfg.kDStart > 0 ? cfg.kDStart : cfg.kP * 2.0;
    double bestCost = 1e18;
    runSearch(cfg, testFn, bestKD, bestCost);
    printFinal("ANGULAR", cfg.kP, bestKD, bestCost);
}

// =========================================================================
//  LINEAR (driving straight) tuner
// =========================================================================

void linearTunerAuton(double kP, double testDistanceIn, double startKD) {
    pros::lcd::initialize();
    g_tuneAborted = false;
    g_stopTuning = false;

    TuneCfg cfg;
    cfg.kP = kP;
    cfg.kDStart = startKD;
    cfg.settleTol = 0.5;     // in
    cfg.overshootTol = 0.75; // in

    double dir = 1.0;

    auto testFn = [&](double kD) -> TuneResult {
        TuneResult r{};
        r.overshoot = 0;
        r.oscillations = 0;
        r.riseTime = cfg.testDuration;
        r.settleTime = cfg.testDuration;
        r.steadyError = 0;

        // see NOTE above angularPID's assignment in the angular tuner
        new (&chassis.lateralPID) lemlib::PID(cfg.kP, 0, kD, 0, false);

        lemlib::Pose start = chassis.getPose();
        double thetaRad = start.theta * (kPi / 180.0);
        // forward unit vector for LemLib's compass heading convention
        // (0deg = +y, increasing clockwise). `dir` folds in alternation.
        double ux = sin(thetaRad) * dir;
        double uy = cos(thetaRad) * dir;
        double targetX = start.x + ux * testDistanceIn;
        double targetY = start.y + uy * testDistanceIn;
        bool forwards = (dir > 0);
        dir = -dir;

        // forwards=false drives the backward tests in reverse instead of
        // spinning the robot 180 degrees first.
        chassis.moveToPoint(targetX, targetY, (int)cfg.testDuration + 500, {.forwards = forwards}, true);

        uint32_t startTime = pros::millis();
        uint32_t elapsed = 0;
        int initialSign = 0;
        int lastSign = 0;
        bool reached = false;
        bool settled = false;
        double settleEnterTime = -1;
        double ssSum = 0;
        int ssCount = 0;

        while (elapsed < (uint32_t)cfg.testDuration) {
            if (tunerKilled()) {
                g_tuneAborted = true;
                break;
            }

            lemlib::Pose now = chassis.getPose();
            // signed progress along the travel direction, in inches
            double progress = (now.x - start.x) * ux + (now.y - start.y) * uy;
            double error = testDistanceIn - progress;
            int s = sign(error);

            if (initialSign == 0 && s != 0) {
                initialSign = s;
                lastSign = s;
            }
            if (s != 0 && lastSign != 0 && s != lastSign) r.oscillations++;
            if (s != 0) lastSign = s;

            if (initialSign != 0 && s == -initialSign) {
                double mag = fabs(error);
                if (mag > r.overshoot) r.overshoot = mag;
            }

            if (!reached && fabs(error) <= cfg.settleTol) {
                reached = true;
                r.riseTime = elapsed;
            }

            if (fabs(error) <= cfg.settleTol) {
                if (settleEnterTime < 0) settleEnterTime = elapsed;
                if (!settled && (elapsed - settleEnterTime) >= cfg.settleHold) {
                    settled = true;
                    r.settleTime = settleEnterTime;
                }
            } else {
                settleEnterTime = -1;
            }

            if (elapsed >= cfg.testDuration - cfg.ssWindow) {
                ssSum += fabs(error);
                ssCount++;
            }

            pros::delay(10);
            elapsed = pros::millis() - startTime;
        }

        chassis.cancelMotion();
        if (ssCount > 0) r.steadyError = ssSum / ssCount;
        return r;
    };

    double bestKD = cfg.kDStart > 0 ? cfg.kDStart : cfg.kP * 2.0;
    double bestCost = 1e18;
    runSearch(cfg, testFn, bestKD, bestCost);
    printFinal("LINEAR", cfg.kP, bestKD, bestCost);
}