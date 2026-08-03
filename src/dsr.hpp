#pragma once
#include "pros/distance.hpp"
#include "lemlib/api.hpp"

// =========================================================================
//  Wall-based pose correction — distance sensor utilities
// =========================================================================
//
// THE CORE IDEA
// ─────────────
//  Odometry drifts. Over 60 seconds of skills your robot will accumulate
//  small position errors — maybe 0.5", maybe 2" — that compound across
//  every subsequent movement. Wall corrections fix this by using a known
//  physical reference (a wall at a fixed coordinate) to re-anchor the
//  position the robot thinks it's at.
//
//  You place 1-3 correction moments in your skills route at natural
//  "I'm near a wall anyway" moments (scoring a goal next to a wall,
//  returning to the loader zone, etc.), and accuracy stays tight all the
//  way to the Midfield endgame at the end.
//
// HOW IT WORKS
// ─────────────
//  1. Robot drives near a wall and stops (or pauses briefly).
//  2. Distance sensor reads how far the sensor face is from the wall.
//  3. We know the wall's exact field coordinate, and we know how far
//     the sensor face is from the robot's geometric center.
//  4. We compute where the robot's CENTER must be, and call
//     chassis.setPose() to correct only the axis perpendicular to
//     the wall. Heading and the parallel axis are left untouched —
//     the IMU is already accurate for heading.
//
//  GEOMETRY (north wall example):
//    Wall is at y = +72.0" (field edge).
//    Sensor reads d = 4.2" (gap between sensor face and wall).
//    Sensor is sensorOffsetIn = 7.0" forward of robot center.
//    → robot center Y = 72.0 - 4.2 - 7.0 = 60.8"
//    Before correction, odometry might have said y = 62.1".
//    After correction: y = 60.8" (true physical position).
//
// COORDINATE SYSTEM (LemLib 0.5)
// ───────────────────────────────
//  • Origin (0, 0) = wherever you call chassis.setPose(0, 0, 0) at auton start.
//  • +X = right (east), +Y = forward (north).
//  • Heading 0° = facing north (+Y), increases clockwise:
//    0° = N, 90° = E, 180° = S, 270° = W.
//  • A standard skills setup puts the red alliance station at the south
//    side of the field, so the robot starts facing north (heading 0).
//    In that case the north field wall is at y = +something, south at
//    y = 0 or negative depending on where you set your origin.
//
//  TIP: Pick your origin early and be consistent. Many teams set
//  setPose(0, 0, 0) at the robot starting tile center — then the field
//  walls are at approximately ±59" to ±65" depending on robot size and
//  where it started. Measure the wall positions by driving to them and
//  reading the raw distance + your starting pose.
//
// SENSOR OFFSET (sensorOffsetIn)
// ────────────────────────────────
//  Measure from the robot's geometric center to the sensing face of the
//  distance sensor, in inches, along the axis the sensor is mounted on.
//
//  Examples:
//    Front-mounted sensor, 8" forward of center  → sensorOffsetIn = +8.0
//    Rear-mounted  sensor, 6" behind  of center  → sensorOffsetIn = -6.0
//    Left-side-mounted sensor, 5" left of center → sensorOffsetIn = -5.0
//      (for a left-facing sensor, "forward" is leftward, so use the sign
//       convention of the wall function you're calling)
//
//  If you don't have calipers handy, measure from the physical center of
//  the robot chassis to the face of the sensor housing — it doesn't need
//  to be millimeter-perfect, just within 0.25".
//
// SAFETY GATES
// ─────────────
//  Every correction function checks:
//    a) Heading is within headingTolDeg of the expected wall direction.
//       If you're 20° off, the correction math is wrong — skip it.
//    b) Distance reading is between minReadIn and maxReadIn.
//       A reading of 0 = sensor error. A reading of 100" = too far from
//       wall for reliable correction. Both are skipped.
//  If either check fails, the function returns false and does NOT modify
//  the pose. Your odometry keeps running as-is.

struct DistanceCorrectionCfg {
    int    port;                  // PROS port number for the pros::Distance sensor
    double sensorOffsetIn;        // inches from robot geometric center to sensor face
                                  // positive = in the direction the sensor faces
    double headingTolDeg = 15.0;  // maximum heading deviation to allow a correction
    double minReadIn     = 1.0;   // minimum plausible distance reading (inches)
    double maxReadIn     = 55.0;  // maximum plausible distance reading (inches)
};

// ── Four directional helpers ──────────────────────────────────────────────
//
// Call these when your robot is facing the indicated wall direction and
// has stopped (or briefly paused) near the wall.
//
// wallCoordIn: the Y (for N/S) or X (for E/W) field coordinate of the
// physical wall surface, in your LemLib coordinate frame.
//
// Returns true if correction was applied, false if skipped.

// Robot is facing NORTH (heading ≈ 0°), sensor pointed at the north wall.
// Corrects the robot's Y position.
bool correctFromNorthWall(lemlib::Chassis& chassis,
                          const DistanceCorrectionCfg& cfg,
                          double wallCoordIn);

// Robot is facing SOUTH (heading ≈ 180°), sensor pointed at the south wall.
// Corrects the robot's Y position.
bool correctFromSouthWall(lemlib::Chassis& chassis,
                          const DistanceCorrectionCfg& cfg,
                          double wallCoordIn);

// Robot is facing EAST (heading ≈ 90°), sensor pointed at the east wall.
// Corrects the robot's X position.
bool correctFromEastWall(lemlib::Chassis& chassis,
                         const DistanceCorrectionCfg& cfg,
                         double wallCoordIn);

// Robot is facing WEST (heading ≈ 270°), sensor pointed at the west wall.
// Corrects the robot's X position.
bool correctFromWestWall(lemlib::Chassis& chassis,
                         const DistanceCorrectionCfg& cfg,
                         double wallCoordIn);

// ── Generic version ───────────────────────────────────────────────────────
//
// Handles any heading. Pass the heading the robot SHOULD be facing
// (degrees, LemLib compass convention). Automatically resolves whether
// to correct X or Y based on the heading.
//
// Use this if your correction happens at a diagonal heading or if you
// want a single function call that handles multiple wall orientations.
bool correctFromWall(lemlib::Chassis& chassis,
                     const DistanceCorrectionCfg& cfg,
                     double expectedHeadingDeg,
                     double wallCoordIn);

// ── Utility: raw distance reading ─────────────────────────────────────────
// Returns the sensor reading in inches, or -1.0 on error.
// Useful for debugging: call this and print to the LCD during testing.
double readDistanceInches(const DistanceCorrectionCfg& cfg);