# Vision System Flowcharts and Diagrams

## Overview

This document contains detailed flowcharts and diagrams illustrating the vision system's operation, decision logic, and data flows.

---

## Table of Contents

1. [System Initialization Flow](#system-initialization-flow)
2. [Periodic Update Flow](#periodic-update-flow)
3. [MegaTag Selection Decision Tree](#megatag-selection-decision-tree)
4. [Vision Measurement Validation](#vision-measurement-validation)
5. [Standard Deviation Calculation](#standard-deviation-calculation)
6. [Error Handling Flow](#error-handling-flow)
7. [Alliance Tag Filter Setup](#alliance-tag-filter-setup)

---

## System Initialization Flow

### Robot Startup Sequence

```
┌─────────────────────────────────────────────────────────────────────┐
│                       ROBOT INITIALIZATION                          │
└─────────────────────────────────────────────────────────────────────┘

Robot.robotInit()
    │
    └──► RobotContainer()
            │
            ├──► Load Field Layout
            │       │
            │       ├─► Try: Load custom_field.json
            │       │      │
            │       │      ├─ Success → Use custom layout
            │       │      │
            │       │      └─ Failure ─┐
            │       │                  │
            │       └─► Catch: Load default (k2025ReefscapeWelded)
            │                  │
            │                  ├─ Success → Use default layout
            │                  │
            │                  └─ Failure → fieldLayout = null ⚠️
            │
            ├──► Create CommandSwerveDrivetrain
            │       └─► Initialize pose estimator
            │
            └──► Create LimelightVisionSubsystem
                    │
                    ├─► For each camera in kLimelights:
                    │      │
                    │      ├─► Create LimelightDevice
                    │      │      ├─ name = "limelight-FrontLL"
                    │      │      ├─ model = LIMELIGHT_3G
                    │      │      ├─ id = 11
                    │      │      └─ role = LOCALIZATION
                    │      │
                    │      ├─► Assign by role:
                    │      │      ├─ LOCALIZATION → frontLL
                    │      │      ├─ CAMERAFEED → (no assignment)
                    │      │      └─ (other) → log warning
                    │      │
                    │      └─► Setup diagnostics:
                    │             ├─ SmartDashboard entries
                    │             └─ Field2d widgets
                    │
                    ├─► Initialize NetworkTables publisher
                    │      └─ visionTargetPublisher
                    │
                    ├─► Update tag filters
                    │      └─► For each device:
                    │             └─ setTagFilter(alliance)
                    │                   └─ LimelightHelpers.SetFiducialIDFiltersOverride()
                    │
                    └─► Subsystem ready ✓
```

---

## Periodic Update Flow

### Main Vision Loop (20ms cycle)

```
┌─────────────────────────────────────────────────────────────────────┐
│                     PERIODIC UPDATE (Every 20ms)                    │
└─────────────────────────────────────────────────────────────────────┘

CommandScheduler.run()
    │
    └──► LimelightVisionSubsystem.periodic()
            │
            ├──► 1. Calculate time deltas
            │       ├─ currentTime = System.currentTimeMillis()
            │       ├─ sinceLastMT1 = currentTime - lastMegaTag1Reading
            │       ├─ sinceLastMT2 = currentTime - lastMegaTag2Reading
            │       └─ wantNewMT1 = (sinceLastMT1 > 10000ms) ?
            │
            ├──► 2. Get all vision measurements
            │       │
            │       └─► getVisionMeasurements()
            │              │
            │              └─► For each LimelightDevice:
            │                     │
            │                     └─► getVisionMeasurement(drivetrain)
            │                            │
            │                            ├─► Get PoseEstimate from NetworkTables
            │                            ├─► Validate quality (see Validation flow)
            │                            ├─► Select method (see Decision Tree)
            │                            ├─► Calculate std devs (see Calculation)
            │                            └─► Return Optional<VisionMeasurement>
            │
            ├──► 3. Process each measurement
            │       │
            │       └─► For each measurement:
            │              │
            │              ├─► First measurement?
            │              │      └─ Yes → Set initalPoseSet = true
            │              │
            │              ├─► Track timing
            │              │      ├─ MegaTag1? → Update lastMegaTag1Reading
            │              │      └─ Set isMegaTag1ReadingNew if wanted
            │              │
            │              ├─► Add to drivetrain
            │              │      └─ drivetrain.addVisionMeasurement(
            │              │            pose, timestamp, stdDevs)
            │              │
            │              └─► Update diagnostics (if enabled)
            │                     ├─ SmartDashboard: stddev, count, distance
            │                     ├─ SmartDashboard: speed, method
            │                     └─ Field2d: pose visualization
            │
            └──► 4. Publish visible tags (if diagnostics enabled)
                   └─► Get visible tag poses
                          │
                          ├─► getVisibleTagIDs() from all cameras
                          ├─► For each ID: fieldLayout.getTagPose(id)
                          └─► visionTargetPublisher.set(poses[])
```

---

## MegaTag Selection Decision Tree

### Method Selection Algorithm

```
┌─────────────────────────────────────────────────────────────────────┐
│                   MEGATAG METHOD SELECTION                          │
└─────────────────────────────────────────────────────────────────────┘

                        getVisionMeasurement()
                                │
                                ▼
                    ┌───────────────────────┐
                    │ Check Role            │
                    │ role == LOCALIZATION? │
                    └───────────┬───────────┘
                                │
                    ┌───────────┴────────────┐
                   NO                       YES
                    │                        │
                    ▼                        ▼
            ┌───────────────┐    ┌────────────────────────┐
            │ Return Empty  │    │ Get PoseEstimate       │
            └───────────────┘    │ from NetworkTables     │
                                 └────────┬───────────────┘
                                          │
                                          ▼
                                 ┌────────────────────────┐
                                 │ validPoseEstimate()?   │
                                 │ (tv == 1, tags > 0)    │
                                 └────────┬───────────────┘
                                          │
                        ┌─────────────────┴─────────────────┐
                       NO                                   YES
                        │                                    │
                        ▼                                    ▼
                ┌───────────────┐              ┌──────────────────────┐
                │ Return Empty  │              │ Check Match State    │
                └───────────────┘              │ BEFORE_MATCH?        │
                                               └──────────┬───────────┘
                                                          │
                                        ┌─────────────────┴─────────────┐
                                       YES                              NO
                                        │                               │
                                        ▼                               ▼
                            ┌────────────────────────┐    ┌──────────────────────┐
                            │ Distance Check         │    │ Distance Check       │
                            │ avgTagDist < 8m?       │    │ avgTagDist < 8m?     │
                            └────────┬───────────────┘    └──────────┬───────────┘
                                     │                               │
                        ┌────────────┴────────────┐       ┌──────────┴──────────┐
                       NO                        YES      NO                    YES
                        │                         │       │                      │
                        ▼                         ▼       ▼                      ▼
                ┌───────────────┐    ┌────────────────────────┐  ┌─────────┐  Continue
                │ Return Empty  │    │ Check Conditions       │  │ Return  │     │
                └───────────────┘    │ CAN_GET_GOOD_HEADING?  │  │ Empty   │     │
                                     └────────┬───────────────┘  └─────────┘     │
                                              │                                   │
                                    ┌─────────┴──────────┐                       │
                                   YES                   NO                      │
                                    │                    │                       │
                                    └────────┬───────────┘                       │
                                             │                                   │
                                             ▼                                   ▼
                                  ┌────────────────────────┐      ┌──────────────────────┐
                                  │ Override Active?       │      │ Check Quality        │
                                  │ getMegaTag1Override()? │      │ - 2+ tags?           │
                                  └────────┬───────────────┘      │ - dist < 3.75m?      │
                                           │                      │ - speed < 0.5 m/s?   │
                        ┌──────────────────┴─────────┐            └──────────┬───────────┘
                       YES                           NO                      │
                        │                             │          ┌────────────┴───────────┐
                        └──────────┬──────────────────┘         ALL                     ANY
                                   │                            MET                     FAIL
                                   ▼                             │                       │
                        ┌──────────────────────┐                │                       │
                        │   USE MEGATAG 1      │◄───────────────┘                       │
                        │                      │                                        │
                        │ - Position ✓         │                                        │
                        │ - Rotation ✓         │                                        │
                        │ - Better heading     │                                        │
                        └──────────┬───────────┘                                        │
                                   │                                                    │
                                   │           ┌────────────────────────────────────────┘
                                   │           │
                                   └─────┬─────┘
                                         │
                                         ▼
                              ┌────────────────────────┐
                              │   USE MEGATAG 2        │
                              │                        │
                              │ - Position ✓           │
                              │ - Rotation ✗ (gyro)    │
                              │ - More robust          │
                              └────────────┬───────────┘
                                           │
                                           ▼
                                ┌────────────────────────┐
                                │ getVisionMeasurement(  │
                                │   swerve, method)      │
                                └────────────┬───────────┘
                                             │
                                             ▼
                                    (Continue to validation)
```

### CAN_GET_GOOD_HEADING Conditions

```
CAN_GET_GOOD_HEADING = 
    twoOrMoreTags        (tagCount >= 2)
    AND closeEnough      (avgTagDist < 3.75m)
    AND movingSlowEnough (robotSpeed < 0.5 m/s)

Example scenarios:
    ✓ 2 tags, 2m, 0.3 m/s   → CAN_GET_GOOD_HEADING = true  → MegaTag1
    ✗ 2 tags, 5m, 0.3 m/s   → CAN_GET_GOOD_HEADING = false → MegaTag2
    ✗ 1 tag,  2m, 0.3 m/s   → CAN_GET_GOOD_HEADING = false → MegaTag2
    ✗ 2 tags, 2m, 0.8 m/s   → CAN_GET_GOOD_HEADING = false → MegaTag2
```

---

## Vision Measurement Validation

### Validation Pipeline

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MEASUREMENT VALIDATION                           │
└─────────────────────────────────────────────────────────────────────┘

getVisionMeasurement(swerve, method)
    │
    ├──► Set robot orientation (for MegaTag2)
    │      └─ LimelightHelpers.SetRobotOrientation(
    │            name, yaw, 0, 0, 0, 0, 0)
    │
    ├──► Get PoseEstimate based on method
    │      ├─ MegaTag1: getBotPoseEstimate_wpiBlue()
    │      └─ MegaTag2: getBotPoseEstimate_wpiBlue_MegaTag2()
    │
    └──► Calculate Standard Deviations
            │
            ├─► MegaTag1: calculateStdDevsMegaTag1()
            │      │
            │      ├─► validPoseEstimate? ────NO───► Return Empty
            │      │         │
            │      │        YES
            │      │         │
            │      ├─► Single tag?
            │      │      │
            │      │     YES
            │      │      │
            │      │      ├─► ambiguity > 0.7? ──YES─► Return Empty
            │      │      ├─► distance > 5m?    ──YES─► Return Empty
            │      │      └─► Add single tag punishment
            │      │
            │      ├─► Calculate base stddev
            │      │      stddev = kInitialValue (0.3)
            │      │            - min(tagCount,4) × kTagCountReward (0.15)
            │      │            + avgTagDist × kAverageDistancePunishment (0.1)
            │      │            + robotSpeed × kRobotSpeedPunishment (0.15)
            │      │            + (singleTag ? kSingleTagPunishment (0.3) : 0)
            │      │
            │      ├─► Apply minimum
            │      │      stddev = max(stddev, 0.05)
            │      │
            │      └─► Return [stddev, stddev, 0.3]
            │
            └─► MegaTag2: calculateStdDevsMegaTag2()
                   │
                   ├─► validPoseEstimate? ────NO───► Return Empty
                   │         │
                   │        YES
                   │         │
                   ├─► Angular velocity check
                   │      │
                   │      └─► |omega| > 720 deg/s? ──YES─► Return Empty
                   │                  │
                   │                 NO
                   │                  │
                   ├─► Distance check
                   │      │
                   │      └─► avgTagDist > 8m? ──YES─► Return Empty
                   │                  │
                   │                 NO
                   │                  │
                   ├─► Calculate stddev
                   │      stddev = kInitialValue (0.2)
                   │            - (tagCount > 1 ? kMultipleTagsBonus (0.05) : 0)
                   │            + avgTagDist × kAverageDistancePunishment (0.075)
                   │            + robotSpeed × kRobotSpeedPunishment (0.25)
                   │
                   ├─► Apply minimum
                   │      stddev = max(stddev, 0.05)
                   │
                   └─► Return [stddev, stddev, MAX_VALUE]
                          (Never trust rotation from MegaTag2)
```

---

## Standard Deviation Calculation

### MegaTag1 Calculation Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│              MEGATAG1 STANDARD DEVIATION CALCULATION                │
└─────────────────────────────────────────────────────────────────────┘

Input:
├─ tagCount (number of visible tags)
├─ avgTagDist (average distance to tags in meters)
├─ robotSpeed (translational speed in m/s)
└─ singleTag (boolean)

Calculation:
                                          
    transStdDev = 0.3                     ← kInitialValue (baseline trust)
                                          
                - 0.15 × min(tagCount, 4) ← Reward for more tags (max 4)
                                          
                + 0.1 × avgTagDist        ← Punishment for distance
                                          
                + 0.15 × robotSpeed       ← Punishment for speed
                                          
                + (singleTag ? 0.3 : 0)   ← Extra punishment for single tag
    
    transStdDev = max(transStdDev, 0.05)  ← Minimum threshold
    
    rotStdDev = 0.3                       ← Fixed rotation stddev

Output: [transStdDev, transStdDev, rotStdDev]

Examples:

    Scenario 1: 4 tags, 2m, 0.3 m/s, multiple tags
        = 0.3 - (0.15×4) + (0.1×2) + (0.15×0.3) + 0
        = 0.3 - 0.6 + 0.2 + 0.045
        = -0.055 → 0.05 (minimum applied)
        Result: HIGH TRUST [0.05, 0.05, 0.3]
    
    Scenario 2: 2 tags, 3m, 0.5 m/s, multiple tags
        = 0.3 - (0.15×2) + (0.1×3) + (0.15×0.5) + 0
        = 0.3 - 0.3 + 0.3 + 0.075
        = 0.375
        Result: MODERATE TRUST [0.375, 0.375, 0.3]
    
    Scenario 3: 1 tag, 4m, 0.2 m/s, single tag
        = 0.3 - (0.15×1) + (0.1×4) + (0.15×0.2) + 0.3
        = 0.3 - 0.15 + 0.4 + 0.03 + 0.3
        = 0.88
        Result: LOW TRUST [0.88, 0.88, 0.3]
```

### MegaTag2 Calculation Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│              MEGATAG2 STANDARD DEVIATION CALCULATION                │
└─────────────────────────────────────────────────────────────────────┘

Input:
├─ tagCount (number of visible tags)
├─ avgTagDist (average distance to tags in meters)
└─ robotSpeed (translational speed in m/s)

Calculation:

    transStdDev = 0.2                      ← kInitialValue (baseline)
                                          
                - (tagCount>1 ? 0.05 : 0)  ← Bonus for multiple tags
                                          
                + 0.075 × avgTagDist       ← Punishment for distance
                                          
                + 0.25 × robotSpeed        ← Punishment for speed
    
    transStdDev = max(transStdDev, 0.05)   ← Minimum threshold
    
    rotStdDev = Double.MAX_VALUE           ← NEVER trust rotation

Output: [transStdDev, transStdDev, MAX_VALUE]

Examples:

    Scenario 1: 3 tags, 2m, 0.2 m/s
        = 0.2 - 0.05 + (0.075×2) + (0.25×0.2)
        = 0.15 + 0.15 + 0.05
        = 0.35
        Result: GOOD TRUST [0.35, 0.35, MAX]
    
    Scenario 2: 1 tag, 5m, 0.8 m/s
        = 0.2 - 0 + (0.075×5) + (0.25×0.8)
        = 0.2 + 0.375 + 0.2
        = 0.775
        Result: LOW TRUST [0.775, 0.775, MAX]
    
    Scenario 3: 2 tags, 1m, stationary
        = 0.2 - 0.05 + (0.075×1) + 0
        = 0.15 + 0.075
        = 0.225
        Result: HIGH TRUST [0.225, 0.225, MAX]
```

---

## Error Handling Flow

### Error Recovery Strategy

```
┌─────────────────────────────────────────────────────────────────────┐
│                       ERROR HANDLING FLOW                           │
└─────────────────────────────────────────────────────────────────────┘

Error Scenarios:

1. Field Layout Load Failure
   │
   ├─► Try: Load custom_field.json
   │      └─► IOException
   │            ├─► Log: "Failed to load custom field"
   │            └─► Try: Load default (k2025ReefscapeWelded)
   │                   └─► IOException
   │                         ├─► Log: Error
   │                         └─► fieldLayout = null ⚠️
   │
   └─► Impact: getVisibleTagPoses() will throw NullPointerException
       Solution: Add null check before use

2. Invalid Pose Estimate
   │
   ├─► Check: validPoseEstimate()
   │      ├─ tv == 0 (no target)
   │      ├─ tagCount == 0
   │      └─ pose == null
   │
   └─► Action: Return Optional.empty()
       Impact: No measurement this cycle (normal behavior)

3. Tag ID Not in Field Layout
   │
   ├─► fieldLayout.getTagPose(tagID) returns Optional.empty()
   │      │
   │      └─► Log warning: "Tag ID X not found in field layout"
   │
   └─► Action: Skip this tag
       Impact: Reduces visible tag count

4. Network Disconnection
   │
   ├─► NetworkTables stops updating
   │      ├─ tv stays at last value
   │      └─ botpose stops updating
   │
   └─► Action: validPoseEstimate() fails
       Impact: Falls back to odometry only

5. Camera Hardware Failure
   │
   ├─► Symptoms:
   │      ├─ No network response
   │      ├─ Constant tv = 0
   │      └─ No camera feed
   │
   └─► Action: No measurements generated
       Impact: Odometry-only operation
```

---

## Alliance Tag Filter Setup

### Tag Filter Configuration Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                   ALLIANCE TAG FILTER SETUP                         │
└─────────────────────────────────────────────────────────────────────┘

Trigger: Alliance change detected OR initialization

updateTagFilters()
    │
    └─► Get alliance from DriverStation
           │
           └─► DriverStation.getAlliance()
                  │
                  ├─► Optional.empty() (no alliance)
                  │      │
                  │      └─► For each limelight:
                  │             └─ setTagFilter([])  (no filtering)
                  │
                  ├─► Red Alliance
                  │      │
                  │      └─► For each limelight:
                  │             │
                  │             ├─ tagFilter = RED_ALLIANCE tags
                  │             │    └─ Currently: [1,2,3,4,5,6] ⚠️ WRONG
                  │             │    └─ Should be: [9,10,11,12,13,14,15,16]
                  │             │
                  │             └─ LimelightHelpers.SetFiducialIDFiltersOverride(
                  │                   name, tagFilter)
                  │                      │
                  │                      └─► NetworkTables:
                  │                             fiducial_id_filters_set = [...]
                  │
                  └─► Blue Alliance
                         │
                         └─► For each limelight:
                                │
                                ├─ tagFilter = BLUE_ALLIANCE tags
                                │    └─ Currently: [1,2,3,4,5,6] ⚠️ WRONG
                                │    └─ Should be: [1,2,3,4,5,6,7,8]
                                │
                                └─ LimelightHelpers.SetFiducialIDFiltersOverride(
                                      name, tagFilter)

Effect on Detection:
    │
    ├─► Limelight hardware filters detections
    │      └─ Only reports tags in filter array
    │
    └─► Result: Only relevant tags used for localization
```

### 2025 Reefscape Field Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│                  2025 REEFSCAPE APRILTAG LAYOUT                     │
└─────────────────────────────────────────────────────────────────────┘

Blue Alliance Side                     Red Alliance Side
(Driver Station)                       (Driver Station)

    Tags 1-8                               Tags 9-11
    
    ┌─────────────────────────────────────────────────┐
    │  1   2   3   4   5   6   7   8                  │ Blue
    │  ●   ●   ●   ●   ●   ●   ●   ●                  │
    │                                                  │
    │                                                  │
    │              🤖 Robot                            │
    │                                                  │
    │                                                  │
    │                                 ●   ●   ●       │ Red
    │                                 9   10  11      │
    └─────────────────────────────────────────────────┘

Alliance Configuration (verify with field documentation):
    Blue Robot: Should only see tags 1-8
    Red Robot: Should only see tags 9-11

Purpose: Prevents cross-field ambiguity

Note: 2025 Reefscape has 11 tags total (IDs 1-11).
Exact positions depend on final field layout.
```

---

## Summary

These flowcharts illustrate:

1. **Initialization**: How the system starts up and configures itself
2. **Periodic Updates**: The main 20ms processing loop
3. **Method Selection**: How MegaTag 1 vs 2 is chosen
4. **Validation**: Quality checks before accepting measurements
5. **Std Dev Calculation**: Trust level computation
6. **Error Handling**: Recovery from common failures
7. **Tag Filtering**: Alliance-based tag selection

**Key Takeaways:**
- System has multiple validation layers
- Automatic method selection based on conditions
- Dynamic trust calculation rewards quality
- Robust error handling (mostly returns empty)
- Alliance filtering prevents cross-field confusion

---

*Flowcharts and Diagrams - Version 1.0*  
*Last Updated: 2026-01-04*
