# LimelightDevice Class

## Overview

`LimelightDevice` is a wrapper class for individual Limelight cameras.  It handles: 
- Communication with the camera via NetworkTables
- AprilTag filtering by field region
- Vision measurement retrieval
- Standard deviation calculation for pose estimation

## File Location

```
src/main/java/com/spartronics4915/frc2025/subsystems/vision/LimelightDevice. java
```

## Class Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           LimelightDevice                                │
│                        extends SubsystemBase                             │
├──────────────────────────────────────────────────────────────────────────┤
│  - name: String                                                          │
│  - model: LimelightModel                                                 │
│  - id: int                                                               │
│  - role: LimelightRole                                                   │
│  - tagFilter: int[]                                                      │
├──────────────────────────────────────────────────────────────────────────┤
│  + LimelightDevice(constants: LimelightConstants)                        │
│  + setTagFilter(alliance: Optional<Alliance>): void                      │
│  + getRole(): LimelightRole                                              │
│  + getTx(): double                                                       │
│  + getTy(): double                                                       │
│  + getTv(): boolean                                                      │
│  + getVisionMeasurement(swerve: SwerveSubsystem): Optional<VisionMeas>   │
│  + getVisionMeasurement(swerve, method): Optional<VisionMeasurement>     │
│  + getPose2d(): Optional<Pose2d>                                         │
│  + getVisibleTags(): ArrayList<Integer>                                  │
│  - calculateStdDevsMegaTag1(... ): Optional<Matrix<N3, N1>>               │
│  - calculateStdDevsMegaTag2(... ): Optional<Matrix<N3, N1>>               │
└──────────────────────────────────────────────────────────────────────────┘
```

## Constructor

```java
public LimelightDevice(LimelightConstants constants)
```

Creates a new LimelightDevice with the given configuration.

| Parameter | Type | Description |
|-----------|------|-------------|
| `constants` | `LimelightConstants` | Record containing name, model, id, and role |

## Key Methods

### `setTagFilter(Optional<DriverStation.Alliance> alliance)`

Configures which AprilTags this camera should look for based on alliance and role.

```
┌─────────────────────────────────────────────────────────────────┐
│                    TAG FILTER SELECTION                         │
└─────────────────────────────────────────────────────────────────┘

          ┌────────────────┐
          │  Camera Role?   │
          └───────┬────────┘
                  │
    ┌─────────────┼─────────────┬─────────────┐
    │             │             │             │
   REEF        ALIGN        STATION       NOTHING
    │             │             │             │
    ▼             ▼             ▼             ▼
┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐
│ Reef   │   │ Reef   │   │Station │   │ Empty  │
│ Tags   │   │ Tags   │   │+ Barge │   │ (none) │
│        │   │        │   │ Tags   │   │        │
└────────┘   └────────┘   └────────┘   └────────┘
    │             │             │             │
    └─────────────┴──────┬──────┴─────────────┘
                         │
                         ▼
          ┌──────────────────────────┐
          │   Filter by Alliance     │
          │  (Red, Blue, or Both)    │
          └──────────────────────────┘
```

### `getVisionMeasurement(SwerveSubsystem swerve)`

Gets a vision measurement, automatically selecting the best pose estimation method.

**Method Selection Logic:**

```
┌─────────────────────────────────────────────────────────────────┐
│              AUTOMATIC METHOD SELECTION                         │
└─────────────────────────────────────────────────────────────────┘

Start with:  MegaTag 2 (default)

Upgrade to MegaTag 1 if ALL conditions met:
  ✓ tagCount >= 2
  ✓ robotSpeed < 0.5 m/s
  ✓ avgTagDistance < 3.75 m

OR if MegaTag1 override is enabled

Special case - Before match starts:
  Only accept MegaTag 1 readings (for initial pose)
```

### Standard Deviation Calculation

#### MegaTag 1

```java
// Base value
transStdDev = 0.3

// Adjustments
transStdDev -= min(tagCount, 4) * 0.15    // More tags = more trust
transStdDev += avgTagDist * 0.1           // Farther = less trust
transStdDev += robotSpeed * 0.15          // Faster = less trust

// Single tag penalty
if (singleTag) transStdDev += 0.3

// Rotation std dev
rotStdDev = 0.3  // Trust rotation from MegaTag1
```

#### MegaTag 2

```java
// Base value
transStdDev = 0.2

// Adjustments
if (tagCount > 1) transStdDev -= 0.05     // Multi-tag bonus
transStdDev += avgTagDist * 0.075         // Distance penalty
transStdDev += robotSpeed * 0.25          // Speed penalty

// Rotation std dev
rotStdDev = Double.MAX_VALUE  // Never trust rotation from MegaTag2
```

## Data Flow

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        LIMELIGHT DEVICE DATA FLOW                            │
└──────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────┐
    │ Limelight Camera│
    │   (Hardware)    │
    └────────┬────────┘
             │ NetworkTables
             ▼
    ┌─────────────────────────────────────────────┐
    │           LimelightHelpers                  │
    │  - getBotPoseEstimate_wpiBlue()             │
    │  - getBotPoseEstimate_wpiBlue_MegaTag2()    │
    └────────────────────┬────────────────────────┘
                         │
                         ▼
    ┌─────────────────────────────────────────────┐
    │           getVisionMeasurement()            │
    │                                             │
    │  1. Check role (skip if NOTHING)            │
    │  2. Get pose estimate from LimelightHelpers │
    │  3.  Validate estimate (distance, etc.)      │
    │  4. Select estimation method                │
    │  5. Calculate standard deviations           │
    │  6. Return VisionMeasurement                │
    └────────────────────┬────────────────────────┘
                         │
                         ▼
    ┌─────────────────────────────────────────────┐
    │             VisionMeasurement               │
    │  - pose: Pose2d                             │
    │  - timestamp: double                        │
    │  - stdDevs: Matrix<N3, N1>                  │
    │  - diagnostic info                          │
    └─────────────────────────────────────────────┘
```

## Limelight Roles

| Role | Purpose | Tag Filter |
|------|---------|------------|
| `NOTHING` | Camera disabled | Empty (no tags) |
| `REEF` | Primary localization at reef | Reef tags only |
| `ALIGN` | Alignment assistance | Reef tags only |
| `STATION` | Coral station detection | Station + Barge tags |

## Camera Configuration

The robot uses 5 Limelight cameras: 

| Name | Model | ID | Role |
|------|-------|-----|------|
| alex | 3G | 11 | NOTHING |
| randy | 3 | 12 | STATION |
| ben | 3G | 13 | REEF |
| chucky | 3 | 14 | NOTHING |
| doug | 3 | 15 | NOTHING |