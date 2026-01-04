# Vision System Architecture

## Overview

This document provides a detailed architectural overview of the vision system implementation for the 2026 Preseason robot.

## System Components

### Component Hierarchy

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           ROBOT CONTAINER                               │
│                                                                         │
│  ┌──────────────────────┐          ┌──────────────────────────────┐    │
│  │  CommandSwerve       │          │  LimelightVision             │    │
│  │  Drivetrain          │◄─────────│  Subsystem                   │    │
│  │                      │          │                              │    │
│  │  - Odometry          │          │  - Manages cameras           │    │
│  │  - Vision fusion     │          │  - Aggregates measurements   │    │
│  │  - Path following    │          │  - Alliance filtering        │    │
│  └──────────────────────┘          └────────────┬─────────────────┘    │
│                                                  │                      │
│                                    ┌─────────────┴───────────┐          │
│                                    │                         │          │
│                          ┌─────────▼──────────┐   ┌─────────▼────────┐ │
│                          │  LimelightDevice   │   │  LimelightDevice │ │
│                          │   (Front LL)       │   │   (Future...)    │ │
│                          │                    │   │                  │ │
│                          │  - Pose estimation │   │  - Pose estimate │ │
│                          │  - Std dev calc    │   │  - Std dev calc  │ │
│                          │  - Tag filtering   │   │  - Tag filtering │ │
│                          └─────────┬──────────┘   └─────────┬────────┘ │
│                                    │                        │          │
│                          ┌─────────▼────────────────────────▼────────┐ │
│                          │         LimelightHelpers                  │ │
│                          │                                           │ │
│                          │  - NetworkTables communication            │ │
│                          │  - JSON parsing                           │ │
│                          │  - Pose estimation utilities              │ │
│                          └───────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
```

### Class Diagram

```
┌──────────────────────────────────────┐
│    LimelightVisionSubsystem          │
├──────────────────────────────────────┤
│ - limelights: ArrayList<Device>     │
│ - frontLL: LimelightDevice           │
│ - drivetrain: CommandSwerveDrivetrain│
│ - fieldLayout: AprilTagFieldLayout   │
│ - initalPoseSet: boolean             │
├──────────────────────────────────────┤
│ + getVisionMeasurements()            │
│ + canSeeTags(name): boolean          │
│ + getVisibleTagIDs(): ArrayList<Int> │
│ + getVisibleTagPoses(): ArrayList    │
│ + periodic()                         │
│ - updateTagFilters()                 │
└────────────┬─────────────────────────┘
             │ manages
             │
             ▼
┌──────────────────────────────────────┐
│       LimelightDevice                │
├──────────────────────────────────────┤
│ - name: String                       │
│ - model: LimelightModel              │
│ - id: int                            │
│ - role: LimelightRole                │
│ - tagFilter: int[]                   │
├──────────────────────────────────────┤
│ + getVisionMeasurement()             │
│ + getTx/Ty/Tv()                      │
│ + getVisibleTagIDs()                 │
│ + setTagFilter()                     │
│ - calculateStdDevsMegaTag1()         │
│ - calculateStdDevsMegaTag2()         │
└──────────────────────────────────────┘

┌──────────────────────────────────────┐
│      VisionMeasurement (Record)      │
├──────────────────────────────────────┤
│ + pose: Pose2d                       │
│ + timestamp: double                  │
│ + stdDevs: Matrix<N3, N1>            │
│ + diagName: String                   │
│ + diagTagCount: int                  │
│ + diagTagDistance: double            │
│ + diagRobotSpeed: double             │
│ + diagMethod: PoseEstimationMethod   │
└──────────────────────────────────────┘
```

## Data Flow

### Vision Measurement Pipeline

```
┌──────────────────────────────────────────────────────────────────────────┐
│                      VISION MEASUREMENT PIPELINE                         │
└──────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────┐
    │  Limelight 3G   │  Hardware
    │   "FrontLL"     │
    └────────┬────────┘
             │
             │ NetworkTables
             ▼
    ┌─────────────────────────────────────────┐
    │      LimelightHelpers                   │  Library Layer
    │                                         │
    │  getBotPoseEstimate_wpiBlue()           │
    │  getBotPoseEstimate_wpiBlue_MegaTag2()  │
    │  SetRobotOrientation()                  │
    │  SetFiducialIDFiltersOverride()         │
    └────────┬────────────────────────────────┘
             │
             │ PoseEstimate
             ▼
    ┌─────────────────────────────────────────┐
    │      LimelightDevice                    │  Device Layer
    │                                         │
    │  1. Get PoseEstimate from LL            │
    │  2. Validate measurement                │
    │  3. Select MegaTag1 or MegaTag2         │
    │  4. Calculate standard deviations       │
    │  5. Create VisionMeasurement            │
    └────────┬────────────────────────────────┘
             │
             │ Optional<VisionMeasurement>
             ▼
    ┌─────────────────────────────────────────┐
    │   LimelightVisionSubsystem              │  Subsystem Layer
    │                                         │
    │  1. Collect from all devices            │
    │  2. Update diagnostics                  │
    │  3. Set initial pose (if needed)        │
    │  4. Pass to drivetrain                  │
    └────────┬────────────────────────────────┘
             │
             │ VisionMeasurement
             ▼
    ┌─────────────────────────────────────────┐
    │   CommandSwerveDrivetrain               │  Integration Layer
    │                                         │
    │  addVisionMeasurement(                  │
    │      pose, timestamp, stdDevs)          │
    │                                         │
    │  → SwerveDrivePoseEstimator             │
    │     (Kalman filter fusion)              │
    └─────────────────────────────────────────┘
```

### MegaTag Selection Logic Flow

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    MEGATAG SELECTION DECISION TREE                       │
└──────────────────────────────────────────────────────────────────────────┘

                        ┌─────────────────────┐
                        │   New Vision Data   │
                        └──────────┬──────────┘
                                   │
                                   ▼
                        ┌──────────────────────┐
                        │  Valid Pose Estimate?│
                        └──────────┬───────────┘
                                   │
                     ┌─────────────┴─────────────┐
                     │                           │
                    NO                          YES
                     │                           │
                     ▼                           ▼
            ┌────────────────┐      ┌─────────────────────────┐
            │ Return Empty   │      │  BEFORE_MATCH?          │
            │                │      │  (auto/teleop started?) │
            └────────────────┘      └──────────┬──────────────┘
                                               │
                                 ┌─────────────┴─────────────┐
                                YES                          NO
                                 │                           │
                                 ▼                           ▼
                    ┌──────────────────────────┐  ┌──────────────────────┐
                    │  High Quality Required   │  │   Distance Check     │
                    │                          │  │   avgTagDist < 8m?   │
                    │  - 2+ tags               │  └──────────┬───────────┘
                    │  - Close (< 3.75m)       │             │
                    │  - Slow (< 0.5 m/s)      │     ┌───────┴───────┐
                    └──────────┬───────────────┘    NO              YES
                               │                     │                │
                               ▼                     ▼                ▼
                    ┌──────────────────────┐  ┌──────────┐    ┌────────────┐
                    │  Quality Met?        │  │  Return  │    │  Continue  │
                    └──────────┬───────────┘  │  Empty   │    │            │
                               │              └──────────┘    └──────┬─────┘
                   ┌───────────┴────────────┐                        │
                  YES                       NO                        │
                   │                        │                         │
                   ▼                        ▼                         ▼
         ┌─────────────────┐      ┌─────────────────┐     ┌──────────────────┐
         │  CAN_GET_GOOD   │      │  Return Empty   │     │  Check Conditions│
         │  _HEADING       │      │                 │     │                  │
         └────────┬────────┘      └─────────────────┘     │  - 2+ tags       │
                  │                                        │  - < 3.75m       │
                  │                                        │  - < 0.5 m/s     │
                  │                                        └────────┬─────────┘
                  │                                                 │
                  │                               ┌─────────────────┴────────┐
                  │                              YES                         NO
                  │                               │                           │
                  └───────────────┬───────────────┘                           │
                                  │                                           │
                                  ▼                                           ▼
                       ┌─────────────────────┐                   ┌──────────────────┐
                       │    MEGATAG 1        │                   │   MEGATAG 2      │
                       │                     │                   │                  │
                       │  - Position ✓       │                   │  - Position ✓    │
                       │  - Rotation ✓       │                   │  - Rotation ✗    │
                       │  - Lower StdDev     │                   │  - Higher StdDev │
                       └─────────┬───────────┘                   └────────┬─────────┘
                                 │                                        │
                                 └────────────────┬───────────────────────┘
                                                  │
                                                  ▼
                                    ┌──────────────────────────┐
                                    │  Calculate Std Devs      │
                                    │  Return VisionMeasurement│
                                    └──────────────────────────┘
```

## Periodic Execution Flow

### Main Loop (20ms cycle)

```
┌──────────────────────────────────────────────────────────────────────────┐
│                       PERIODIC EXECUTION (20ms)                          │
└──────────────────────────────────────────────────────────────────────────┘

CommandScheduler.run()
    │
    ├──► RobotContainer (all subsystems)
    │
    └──► LimelightVisionSubsystem.periodic()
            │
            ├─► 1. Calculate time deltas
            │      - sinceLastMegatag1Reading
            │      - sinceLastMegatag2Reading
            │      - wantNewMegaTag1Reading (> 10 seconds)
            │
            ├─► 2. Get all vision measurements
            │      └─► getVisionMeasurements()
            │             │
            │             └─► For each LimelightDevice:
            │                    └─► getVisionMeasurement(drivetrain)
            │                           │
            │                           ├─► Get PoseEstimate
            │                           ├─► Validate quality
            │                           ├─► Select MegaTag method
            │                           ├─► Calculate std devs
            │                           └─► Return measurement
            │
            ├─► 3. Process each measurement
            │      │
            │      ├─► Set initial pose (first time only)
            │      │
            │      ├─► Track MegaTag1 readings
            │      │
            │      ├─► Add to drivetrain
            │      │      └─► drivetrain.addVisionMeasurement(
            │      │              pose, timestamp, stdDevs)
            │      │
            │      └─► Update diagnostics (if enabled)
            │             ├─► SmartDashboard: stddev
            │             ├─► SmartDashboard: tag count
            │             ├─► SmartDashboard: distance
            │             ├─► SmartDashboard: speed
            │             ├─► SmartDashboard: method
            │             └─► Field2d: pose
            │
            └─► 4. Publish visible tags (if diagnostics)
                   └─► visionTargetPublisher.set(visibleTagPoses)
```

## Configuration Flow

### Initialization Sequence

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        INITIALIZATION SEQUENCE                           │
└──────────────────────────────────────────────────────────────────────────┘

Robot.robotInit()
    │
    └──► RobotContainer()
            │
            ├──► 1. Load AprilTag Field Layout
            │       │
            │       ├─► Try: custom_field.json
            │       │      └─► ❌ FAILS (file missing)
            │       │
            │       └─► Catch: Load default field
            │              └─► k2025ReefscapeWelded
            │
            ├──► 2. Create CommandSwerveDrivetrain
            │       └─► TunerConstants.createDrivetrain()
            │
            ├──► 3. Create LimelightVisionSubsystem
            │       │
            │       └─► For each limelight in kLimelights:
            │              │
            │              ├─► Create LimelightDevice(constants)
            │              │      ├─► name = "limelight-FrontLL"
            │              │      ├─► model = LIMELIGHT_3G
            │              │      ├─► id = 11
            │              │      └─► role = LOCALIZATION
            │              │
            │              ├─► Assign role:
            │              │      └─► LOCALIZATION → frontLL = device
            │              │
            │              ├─► Setup diagnostics (if enabled)
            │              │      ├─► SmartDashboard entries
            │              │      └─► Field2d widgets
            │              │
            │              └─► Add to limelights array
            │
            ├──► 4. Setup NetworkTables publisher
            │       └─► visionTargetPublisher (for visible tags)
            │
            ├──► 5. Update tag filters
            │       └─► updateTagFilters()
            │              └─► For each device:
            │                     └─► setTagFilter(alliance)
            │                            └─► LimelightHelpers.
            │                                  SetFiducialIDFiltersOverride()
            │
            ├──► 6. Configure swerve bindings
            │
            └──► 7. Configure misc bindings
```

### Alliance-Based Tag Filtering

```
┌──────────────────────────────────────────────────────────────────────────┐
│                      ALLIANCE TAG FILTERING                              │
└──────────────────────────────────────────────────────────────────────────┘

DriverStation.getAlliance()
    │
    ├──► Red Alliance
    │       │
    │       └─► setTagFilter([1, 2, 3, 4, 5, 6])  ⚠️ INCORRECT
    │              └─► LimelightHelpers.SetFiducialIDFiltersOverride()
    │                     └─► NetworkTables: fiducial_id_filters_set
    │
    ├──► Blue Alliance
    │       │
    │       └─► setTagFilter([1, 2, 3, 4, 5, 6])  ⚠️ INCORRECT (same as red!)
    │              └─► LimelightHelpers.SetFiducialIDFiltersOverride()
    │
    └──► No Alliance
            │
            └─► setTagFilter([])  (no filtering)


Expected Configuration:
    Red Alliance  → [9, 10, 11] (verify actual field layout)
    Blue Alliance → [1, 2, 3, 4, 5, 6, 7, 8]
```

## Standard Deviation Calculation

### MegaTag 1 Algorithm

```
transStdDev = kInitialValue (0.3)

IF single tag detected:
    IF ambiguity > 0.7 OR distance > 5m:
        REJECT measurement
    ELSE:
        transStdDev += kSingleTagPunishment (0.3)

transStdDev -= min(tagCount, 4) × kTagCountReward (0.15)
transStdDev += avgTagDist × kAverageDistancePunishment (0.1)
transStdDev += robotSpeed × kRobotSpeedPunishment (0.15)

transStdDev = max(transStdDev, 0.05)  // minimum trust

rotStdDev = 0.3  // fixed for MegaTag1

RETURN [transStdDev, transStdDev, rotStdDev]


Examples:
    2 tags, 2m away, 0.2 m/s speed:
        = 0.3 - (2 × 0.15) + (2 × 0.1) + (0.2 × 0.15)
        = 0.3 - 0.3 + 0.2 + 0.03
        = 0.23

    4 tags, 1m away, stationary:
        = 0.3 - (4 × 0.15) + (1 × 0.1) + 0
        = 0.3 - 0.6 + 0.1
        = max(-0.2, 0.05)
        = 0.05  (minimum)

    1 tag, 3m away, 1 m/s speed:
        = 0.3 + 0.3 - (1 × 0.15) + (3 × 0.1) + (1 × 0.15)
        = 0.6 - 0.15 + 0.3 + 0.15
        = 0.9  (low trust)
```

### MegaTag 2 Algorithm

```
IF angular_velocity > 720 deg/s:
    REJECT measurement

IF avgTagDist > 8m:
    REJECT measurement

transStdDev = kInitialValue (0.2)

IF tagCount > 1:
    transStdDev -= kMultipleTagsBonus (0.05)

transStdDev += avgTagDist × kAverageDistancePunishment (0.075)
transStdDev += robotSpeed × kRobotSpeedPunishment (0.25)

transStdDev = max(transStdDev, 0.05)  // minimum trust

rotStdDev = Double.MAX_VALUE  // NEVER trust rotation

RETURN [transStdDev, transStdDev, rotStdDev]


Examples:
    2 tags, 2m away, 0.2 m/s speed:
        = 0.2 - 0.05 + (2 × 0.075) + (0.2 × 0.25)
        = 0.15 + 0.15 + 0.05
        = 0.35

    1 tag, 5m away, 0.5 m/s speed:
        = 0.2 + (5 × 0.075) + (0.5 × 0.25)
        = 0.2 + 0.375 + 0.125
        = 0.70  (lower trust)
```

## Integration Points

### Drivetrain Integration

```java
// In LimelightVisionSubsystem.periodic()
getVisionMeasurements().forEach((measurement) -> {
    drivetrain.addVisionMeasurement(
        measurement.pose(),           // Pose2d
        measurement.timestamp(),      // double (FPGA time)
        measurement.stdDevs()         // Matrix<N3, N1>
    );
});

// In CommandSwerveDrivetrain
public void addVisionMeasurement(
    Pose2d visionRobotPoseMeters,
    double timestampSeconds,
    Matrix<N3, N1> visionMeasurementStdDevs
) {
    super.addVisionMeasurement(
        visionRobotPoseMeters,
        Utils.fpgaToCurrentTime(timestampSeconds),
        visionMeasurementStdDevs
    );
}
```

### NetworkTables Interface

```
NetworkTables Structure:
    /limelight-FrontLL/
        ├─ tx              (target X offset)
        ├─ ty              (target Y offset)
        ├─ tv              (has valid target)
        ├─ botpose_wpiblue (MegaTag1 pose)
        ├─ botpose_orb_wpiblue_megatag2 (MegaTag2 pose)
        ├─ fiducial_id_filters_set (tag filter)
        └─ robot_orientation_set (gyro data for MT2)

SmartDashboard Diagnostics:
    /SmartDashboard/
        ├─ VisionDiagnostics/
        │   ├─ limelight-FrontLL/
        │   │   ├─ stddev
        │   │   ├─ count
        │   │   ├─ distance
        │   │   ├─ speed
        │   │   ├─ method
        │   │   └─ pose (Field2d)
        │   ├─ Want New MT1 Reading?
        │   └─ vision targets (Pose3d[])
        ├─ Initial Pose Set?
        ├─ Since Last Megatag1 Reading
        └─ Since Last Megatag2 Reading
```

## Performance Characteristics

### Timing Analysis

```
┌────────────────────────────────────────────────────────────┐
│                    TIMING BREAKDOWN                        │
└────────────────────────────────────────────────────────────┘

Main Loop Cycle: 20ms (50 Hz)
    │
    ├─ Network Tables Read: ~1-2ms
    │   └─ LimelightHelpers.getBotPoseEstimate()
    │
    ├─ Validation & Filtering: <0.1ms
    │   └─ Distance checks, tag count, etc.
    │
    ├─ Std Dev Calculation: <0.1ms
    │   └─ Simple arithmetic
    │
    ├─ Pose Estimator Update: ~0.5ms
    │   └─ Kalman filter fusion
    │
    └─ Diagnostics Publishing: ~0.5-1ms
        └─ SmartDashboard updates

Total Vision Processing: ~2-4ms per cycle
Budget Remaining: 16-18ms for other subsystems
```

### Update Rates

```
Component                   Rate        Notes
─────────────────────────────────────────────────────────────
Limelight Frame Rate        90 Hz       Hardware limit (LL3G)
NetworkTables Updates       100 Hz      Limelight default
Robot Periodic              50 Hz       20ms loop
Vision Measurements         0-50 Hz     Depends on tag visibility
MegaTag1 Throttle          0.1 Hz      Every 10 seconds (config)
```

## Summary

The vision system architecture follows a clean layered design:

1. **Hardware Layer**: Limelight cameras
2. **Communication Layer**: LimelightHelpers + NetworkTables
3. **Device Layer**: LimelightDevice (validation, estimation, filtering)
4. **Subsystem Layer**: LimelightVisionSubsystem (aggregation, diagnostics)
5. **Integration Layer**: CommandSwerveDrivetrain (sensor fusion)

Key strengths:
- Clean separation of concerns
- Flexible MegaTag selection
- Dynamic trust calculation
- Good diagnostic support

Key areas for improvement:
- Fix critical configuration bugs
- Add simulation support
- Improve error logging
- Add comprehensive testing
