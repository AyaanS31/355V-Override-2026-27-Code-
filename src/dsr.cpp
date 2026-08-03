#include "dsr.hpp"
#include "main.h"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

static constexpr double kDegToRad = 3.14159265358979 / 180.0;

// Read the distance sensor and return inches. Returns -1 on failure.
double readDistanceInches(const DistanceCorrectionCfg& cfg) {
    pros::Distance sensor(cfg.port);
    int mm = sensor.get();
    // pros::Distance::get() returns mm, or PROS_ERR (INT32_MAX) on failure.
    if (mm <= 0 || mm == INT32_MAX) return -1.0;
    return static_cast<double>(mm) / 25.4;
}

// Shortest signed angular difference: result in [-180, +180].
static double angleDiff(double a, double b) {
    double d = std::fmod(a - b + 540.0, 360.0) - 180.0;
    return d;
}

// Core correction engine.
// expectedHeadingDeg: compass heading the robot should be facing.
// wallCoordIn:        coordinate of the wall surface along the corrected axis.
// correctX:           true = fix X position, false = fix Y position.
// wallIsPositive:     true if the wall is in the + direction of the corrected axis.
static bool applyCorrectionInternal(lemlib::Chassis& chassis,
                                    const DistanceCorrectionCfg& cfg,
                                    double expectedHeadingDeg,
                                    double wallCoordIn,
                                    bool correctX,
                                    bool wallIsPositive) {
    // 1. Safety check: heading
    lemlib::Pose pose = chassis.getPose();
    double headingError = std::fabs(angleDiff(pose.theta, expectedHeadingDeg));
    if (headingError > cfg.headingTolDeg) {
        pros::lcd::print(7, "[DIST] skipped: heading err %.1f deg", headingError);
        return false;
    }

    // 2. Read sensor
    double d = readDistanceInches(cfg);
    if (d < cfg.minReadIn || d > cfg.maxReadIn) {
        pros::lcd::print(7, "[DIST] skipped: bad read %.2f in", d);
        return false;
    }

    // 3. Compute corrected coordinate.
    //
    //  If the wall is in the positive direction (wallIsPositive = true):
    //    sensorFace = robotCenter + sensorOffset  (along that axis)
    //    d = wallCoord - sensorFace
    //    → robotCenter = wallCoord - d - sensorOffset
    //
    //  If the wall is in the negative direction (wallIsPositive = false):
    //    sensorFace = robotCenter - sensorOffset
    //    d = sensorFace - wallCoord
    //    → robotCenter = wallCoord + d + sensorOffset
    //
    double corrected;
    if (wallIsPositive) {
        corrected = wallCoordIn - d - cfg.sensorOffsetIn;
    } else {
        corrected = wallCoordIn + d + cfg.sensorOffsetIn;
    }

    // 4. Apply. Only update the corrected axis — leave the other axis and
    //    heading exactly as odometry reports them.
    if (correctX) {
        pros::lcd::print(7, "[DIST] X: %.2f -> %.2f in", pose.x, corrected);
        chassis.setPose(corrected, pose.y, pose.theta);
    } else {
        pros::lcd::print(7, "[DIST] Y: %.2f -> %.2f in", pose.y, corrected);
        chassis.setPose(pose.x, corrected, pose.theta);
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API — directional helpers
// ─────────────────────────────────────────────────────────────────────────────

// NORTH wall: robot faces 0°, wall is in +Y direction, we correct Y.
bool correctFromNorthWall(lemlib::Chassis& chassis,
                          const DistanceCorrectionCfg& cfg,
                          double wallCoordIn) {
    return applyCorrectionInternal(chassis, cfg,
                                   0.0,          // expected heading
                                   wallCoordIn,
                                   false,         // correct Y
                                   true);         // wall is in +Y direction
}

// SOUTH wall: robot faces 180°, wall is in -Y direction, we correct Y.
bool correctFromSouthWall(lemlib::Chassis& chassis,
                          const DistanceCorrectionCfg& cfg,
                          double wallCoordIn) {
    return applyCorrectionInternal(chassis, cfg,
                                   180.0,
                                   wallCoordIn,
                                   false,         // correct Y
                                   false);        // wall is in -Y direction
}

// EAST wall: robot faces 90°, wall is in +X direction, we correct X.
bool correctFromEastWall(lemlib::Chassis& chassis,
                         const DistanceCorrectionCfg& cfg,
                         double wallCoordIn) {
    return applyCorrectionInternal(chassis, cfg,
                                   90.0,
                                   wallCoordIn,
                                   true,          // correct X
                                   true);         // wall is in +X direction
}

// WEST wall: robot faces 270°, wall is in -X direction, we correct X.
bool correctFromWestWall(lemlib::Chassis& chassis,
                         const DistanceCorrectionCfg& cfg,
                         double wallCoordIn) {
    return applyCorrectionInternal(chassis, cfg,
                                   270.0,
                                   wallCoordIn,
                                   true,          // correct X
                                   false);        // wall is in -X direction
}

// ─────────────────────────────────────────────────────────────────────────────
//  Generic version — resolves axis and direction from heading automatically
// ─────────────────────────────────────────────────────────────────────────────

bool correctFromWall(lemlib::Chassis& chassis,
                     const DistanceCorrectionCfg& cfg,
                     double expectedHeadingDeg,
                     double wallCoordIn) {
    // Normalize heading to [0, 360)
    double h = std::fmod(expectedHeadingDeg + 360.0, 360.0);

    // Determine correction axis and wall sign from heading quadrant.
    // We allow 45° on either side of the four cardinal directions.
    bool correctX;
    bool wallIsPositive;

    if (h < 45.0 || h >= 315.0) {
        // Facing NORTH → correct Y, wall in +Y
        correctX = false;
        wallIsPositive = true;
    } else if (h >= 45.0 && h < 135.0) {
        // Facing EAST → correct X, wall in +X
        correctX = true;
        wallIsPositive = true;
    } else if (h >= 135.0 && h < 225.0) {
        // Facing SOUTH → correct Y, wall in -Y
        correctX = false;
        wallIsPositive = false;
    } else {
        // Facing WEST → correct X, wall in -X
        correctX = true;
        wallIsPositive = false;
    }

    return applyCorrectionInternal(chassis, cfg,
                                   expectedHeadingDeg,
                                   wallCoordIn,
                                   correctX,
                                   wallIsPositive);
}