# LimelightVisionSubsystem Class

## Overview

`LimelightVisionSubsystem` is the main coordinator for all Limelight cameras on the robot. It:
- Manages multiple `LimelightDevice` instances
- Aggregates vision measurements from all cameras
- Handles alliance-based tag filtering
- Feeds measurements to the swerve drive pose estimator
- Publishes diagnostics for debugging

## File Location

```
src/main/java/com/spartronics4915/frc2025/subsystems/vision/LimelightVisionSubsystem.java
```

## Class Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        LimelightVisionSubsystem                              │
│          extends SubsystemBase                                               │
│          implements VisionDeviceSubsystem, ModeSwitchInterface               │
├──────────────────────────────────────────────────────────────────────────────┤
│  - limelights: ArrayList<LimelightDevice>                                    │
│  - reefLL: LimelightDevice                                                   │
│  - alignLL: LimelightDevice                                                  │
│  - stationLL: LimelightDevice                                                │
│  - swerveSubsystem: SwerveSubsystem                                          │
│  - fieldLayout: AprilTagFieldLayout                                          │
│  - visionTargetPublisher: StructArrayPublisher<Pose3d>                       │
│  - initalPoseSet: boolean                                                    │
│  - lastMegaTag1Reading: long                                                 │
│  - static mt1Override: boolean                                               │
│  - static discardMeasurements: boolean                                       │
├──────────────────────────────────────────────────────────────────────────────┤
│  + LimelightVisionSubsystem(swerve, fieldLayout)                             │
│  + getVisionMeasurements(): ArrayList<VisionMeasurement>                     │
│  + canSeeTags(): boolean                                                     │
│  + getVisibleTagIDs(): ArrayList<Integer>                                    │
│  + getVisibleTagPoses(): ArrayList<Pose3d>                                   │
│  + periodic(): void                                                          │
│  + onModeSwitch(): void                                                      │
│  + isInitialPoseSet(): boolean                                               │
│  + newMegaTag1Reading(): boolean                                             │
│  + getBotPose2dFromReefCamera(): Optional<Pose2d>                            │
│  + getReefLimelight(): LimelightDevice                                       │
│  + getAlignLimelight(): LimelightDevice                                      │
│  + getStationLimelight(): LimelightDevice                                    │
│  + static getMegaTag1Override(): boolean                                     │
│  + static setMegaTag1Override(boolean): void                                 │
│  + static setDiscardMeasurements(boolean): void                              │
│  - updateTagFilters(): void                                                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Architecture

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                      LIMELIGHT VISION SUBSYSTEM                                │
└────────────────────────────────────────────────────────────────────────────────┘

                    ┌────────────────────────────────┐
                    │   LimelightVisionSubsystem     │
                    │                                │
                    │  ┌──────────────────────────┐  │
                    │  │   ArrayList<Limelight>   │  │
                    │  └──────────────────────────┘  │
                    │           │                    │
                    │           ▼                    │
                    │  ┌─────┬─────┬─────┬─────┬─────┐
                    │  │alex │randy│ ben │chuck│doug │
                    │  │ LL  │ LL  │ LL  │ LL  │ LL  │
                    │  │ 11  │ 12  │ 13  │ 14  │ 15  │
                    │  │NONE │STAT │REEF │NONE │NONE │
                    │  └─────┴─────┴─────┴─────┴─────┘
                    │                                │
                    │  Special References:           │
                    │  ┌─────────┬──────────┬──────┐ │
                    │  │ reefLL  │ stationLL│alignLL│ │
                    │  │  (ben)  │ (randy)  │(null)│ │
                    │  └─────────┴──────────┴──────┘ │
                    └────────────────────────────────┘
```

## Periodic Update Flow

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                           PERIODIC() EXECUTION                                 │
└────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────┐
│        periodic() called        │
│    (every robot loop ~20ms)     │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│  Check if new MT1 reading needed│
│  (threshold: 10 seconds)        │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│   getVisionMeasurements()       │
│   - Query all LimelightDevices  │
│   - Collect valid measurements  │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│   For each measurement:         │
│                                 │
│   1. Track initial pose set     │
│   2. Update MT1 reading time    │
│   3. Feed to swerve estimator   │
│   4.  Publish diagnostics        │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│  Publish visible tag poses      │
│  (for visualization)            │
└─────────────────────────────────┘
```

## Key Methods

### `getVisionMeasurements()`

Collects vision measurements from all active Limelight cameras. 

```java
public ArrayList<VisionMeasurement> getVisionMeasurements() {
    ArrayList<VisionMeasurement> measurements = new ArrayList<>();
    limelights.forEach((limelight) -> {
        Optional<VisionMeasurement> measurement = 
            limelight.getVisionMeasurement(swerveSubsystem);
        if (measurement.isPresent()) {
            measurements. add(measurement.get());
        }
    });
    return measurements;
}
```

### `onModeSwitch()`

Called when the robot mode changes (disabled → auto, auto → teleop, etc.). Updates tag filters based on current alliance.

```java
@Override
public void onModeSwitch() {
    updateTagFilters();  // Re-apply alliance-specific tag filters
}
```

### `setMegaTag1Override(boolean)`

Forces all cameras to use MegaTag 1 pose estimation, regardless of conditions.

| Value | Effect |
|-------|--------|
| `true` | Always use MegaTag 1 (includes rotation) |
| `false` | Auto-select based on conditions |

### `setDiscardMeasurements(boolean)`

Controls whether vision measurements are actually fed to the swerve estimator.

| Value | Effect |
|-------|--------|
| `true` | Ignore all vision data |
| `false` | Feed measurements to estimator |

## Diagnostics

When `VisionConstants.kVisionDiagnostics` is enabled, the subsystem publishes:

| Key | Value |
|-----|-------|
| `VisionDiagnostics/limelight-{name}/stddev` | Current standard deviation |
| `VisionDiagnostics/limelight-{name}/count` | Number of visible tags |
| `VisionDiagnostics/limelight-{name}/distance` | Average tag distance |
| `VisionDiagnostics/limelight-{name}/speed` | Robot speed at measurement |
| `VisionDiagnostics/limelight-{name}/method` | MEGATAG_1 or MEGATAG_2 |
| `VisionDiagnostics/limelight-{name}/pose` | Field2d visualization |
| `VisionDiagnostics/vision targets` | 3D poses of visible tags |
| `Initial Pose Set? ` | Whether initial pose was received |
| `VisionDiagnostics/Want New MT1 Reading?` | True if MT1 needed |

## Integration with SwerveSubsystem

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                     VISION → SWERVE INTEGRATION                              │
└──────────────────────────────────────────────────────────────────────────────┘

    LimelightVisionSubsystem                    SwerveSubsystem
    ┌─────────────────────────┐                ┌─────────────────────────┐
    │                         │                │                         │
    │  periodic() {           │                │                         │
    │    measurements =       │                │                         │
    │      getVisionMeasure-  │                │                         │
    │      ments();           │                │                         │
    │                         │                │                         │
    │    for (m :  measurements│                │                         │
    │    ) {                  │                │                         │
    │      swerveSubsystem    │────────────────▶  addVisionMeasurement( │
    │        .addVision-      │  pose,         │    pose,                │
    │         Measurement()   │  timestamp,    │    timestamp,           │
    │    }                    │  stdDevs       │    stdDevs)             │
    │  }                      │                │                         │
    │                         │                │  → Kalman filter fusion │
    └─────────────────────────┘                └─────────────────────────┘
```