# Vision System Overview

This document provides a comprehensive overview of the vision system used in the 2025 Reefscape robot for Team Spartronics 4915.

## Table of Contents

- [Introduction](#introduction)
- [System Architecture](#system-architecture)
- [Data Flow](#data-flow)
- [Pose Estimation Methods](#pose-estimation-methods)
- [Component Overview](#component-overview)
- [Configuration](#configuration)

## Introduction

The vision system is responsible for: 
- **Robot localization** using AprilTag detection
- **Pose estimation** for autonomous and teleop operations
- **Field element detection** (Reef, Station, Barge)
- **Sensor fusion** with swerve drive odometry

The system supports both real hardware (Limelight cameras) and simulation (PhotonVision).

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           VISION SYSTEM ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        VisionDeviceSubsystem                         │   │
│  │                           (Interface)                                │   │
│  │  ┌─────────────────────────┐  ┌─────────────────────────┐           │   │
│  │  │ getVisibleTagIDs()      │  │ getBotPose2dFromReef()  │           │   │
│  │  └─────────────────────────┘  └─────────────────────────┘           │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│              │                                  │                            │
│              ▼                                  ▼                            │
│  ┌──────────────────────────┐    ┌──────────────────────────┐               │
│  │  LimelightVisionSubsystem │    │   SimVisionSubsystem     │               │
│  │      (Real Robot)         │    │     (Simulation)         │               │
│  │                           │    │                          │               │
│  │  ┌─────────────────────┐  │    │  ┌────────────────────┐  │               │
│  │  │   LimelightDevice   │  │    │  │   PhotonCamera     │  │               │
│  │  │   (alex, randy,     │  │    │  │   PhotonPoseEst.    │  │               │
│  │  │    ben, chucky,     │  │    │  │   VisionSystemSim  │  │               │
│  │  │    doug)            │  │    │  └────────────────────┘  │               │
│  │  └─────────────────────┘  │    │                          │               │
│  └──────────────────────────┘    └──────────────────────────┘               │
│              │                                  │                            │
│              └──────────────────┬───────────────┘                            │
│                                 ▼                                            │
│                    ┌─────────────────────────┐                               │
│                    │    OdometrySubsystem    │                               │
│                    │                         │                               │
│                    │  - Fuses vision + swerve│                               │
│                    │  - Provides final pose  │                               │
│                    └─────────────────────────┘                               │
│                                 │                                            │
│                                 ▼                                            │
│                    ┌─────────────────────────┐                               │
│                    │     SwerveSubsystem     │                               │
│                    │  addVisionMeasurement() │                               │
│                    └─────────────────────────┘                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                            VISION DATA FLOW                                   │
└───────────────────────────────────────────────────────────────────────────────┘

    ┌──────────────┐         ┌──────────────┐         ┌──────────────┐
    │  Limelight   │         │  Limelight   │         │  Limelight   │
    │    Cameras   │         │   (Reef)     │         │  (Station)   │
    │  (Hardware)  │         │    "ben"     │         │   "randy"    │
    └──────┬───────┘         └──────┬───────┘         └──────┬───────┘
           │                        │                        │
           ▼                        ▼                        ▼
    ┌──────────────────────────────────────────────────────────────┐
    │                    LimelightHelpers                          │
    │  - getBotPoseEstimate_wpiBlue()                              │
    │  - getBotPoseEstimate_wpiBlue_MegaTag2()                     │
    │  - SetRobotOrientation()                                     │
    │  - SetFiducialIDFiltersOverride()                            │
    └──────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────────┐
    │                     LimelightDevice                          │
    │  - Wraps individual Limelight camera                         │
    │  - Calculates standard deviations                            │
    │  - Filters by AprilTag regions                               │
    │  - Returns VisionMeasurement                                 │
    └──────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────────┐
    │                 LimelightVisionSubsystem                     │
    │  - Manages multiple LimelightDevices                         │
    │  - Aggregates measurements from all cameras                  │
    │  - Updates tag filters based on alliance                     │
    │  - Publishes diagnostics to SmartDashboard                   │
    └──────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
    ┌──────────────────────────────────────────────────────────────┐
    │                    SwerveSubsystem                           │
    │  - addVisionMeasurement(pose, timestamp, stdDevs)            │
    │  - Fuses vision data with wheel odometry                     │
    │  - Uses Kalman filter for sensor fusion                      │
    └──────────────────────────────────────────────────────────────┘
```

## Pose Estimation Methods

The system supports two AprilTag pose estimation methods:

### MegaTag 1
- Uses raw AprilTag detections
- Can estimate both position AND rotation
- Best when:  2+ tags visible, robot moving slowly, close to tags
- Conditions for use: 
  - Tag count ≥ 2
  - Robot speed < 0.5 m/s
  - Average tag distance < 3. 75 m

### MegaTag 2  
- Uses robot's gyro orientation as a prior
- Only estimates position (rotation from gyro)
- More robust when conditions for MegaTag 1 aren't met
- Rejects data when:
  - Angular velocity > 720 deg/s
  - Average tag distance > 8 m

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                    POSE ESTIMATION METHOD SELECTION                           │
└───────────────────────────────────────────────────────────────────────────────┘

                        ┌─────────────────────┐
                        │   New Vision Data   │
                        └──────────┬──────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │  Tags ≥ 2 AND Speed < 0.5m/s │
                    │  AND Distance < 3.75m?        │
                    └──────────────┬───────────────┘
                                   │
                      ┌────────────┴────────────┐
                      │                         │
                     YES                        NO
                      │                         │
                      ▼                         ▼
            ┌─────────────────┐       ┌─────────────────┐
            │    MegaTag 1    │       │    MegaTag 2    │
            │                 │       │                 │
            │ - Position ✓    │       │ - Position ✓    │
            │ - Rotation ✓    │       │ - Rotation ✗    │
            │ - Higher trust  │       │ - Uses gyro     │
            └─────────────────┘       └─────────────────┘
                      │                         │
                      └────────────┬────────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │   Calculate Standard Devs    │
                    │   (Trust level for fusion)   │
                    └──────────────────────────────┘
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │   Add to Swerve Estimator    │
                    └──────────────────────────────┘
```

## Component Overview

| Component | Purpose | Key Methods |
|-----------|---------|-------------|
| `VisionDeviceSubsystem` | Interface for vision devices | `getVisibleTagIDs()`, `getBotPose2dFromReefCamera()` |
| `LimelightVisionSubsystem` | Manages real Limelight cameras | `getVisionMeasurements()`, `canSeeTags()` |
| `LimelightDevice` | Individual camera wrapper | `getVisionMeasurement()`, `getTx/Ty/Tv()` |
| `SimVisionSubsystem` | Simulation vision support | PhotonVision simulation |
| `OdometrySubsystem` | Sensor fusion coordinator | `getPose()` |
| `LimelightHelpers` | Limelight API utilities | Network table access, pose parsing |
| `AprilTagRegion` | Tag filtering by field region | `kReef`, `kStation`, `kBarge` |

## Configuration

### Limelight Cameras

```java
LimelightConstants[] kLimelights = {
    new LimelightConstants("alex",   LimelightModel.LIMELIGHT_3G, 11, LimelightRole. NOTHING),
    new LimelightConstants("randy",  LimelightModel.LIMELIGHT_3,  12, LimelightRole.STATION),
    new LimelightConstants("ben",    LimelightModel. LIMELIGHT_3G, 13, LimelightRole.REEF),
    new LimelightConstants("chucky", LimelightModel.LIMELIGHT_3,  14, LimelightRole. NOTHING),
    new LimelightConstants("doug",   LimelightModel. LIMELIGHT_3,  15, LimelightRole.NOTHING)
};
```

### Vision Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `kMaxAngularSpeed` | 720 deg/s | Max angular velocity for MegaTag2 |
| `kMaxSpeedForMegaTag1` | 0.5 m/s | Max robot speed for MegaTag1 |
| `kMaxDistanceForMegaTag1` | 3.75 m | Max tag distance for MegaTag1 |
| `kMaxDistance` | 8 m | Maximum usable tag distance |

### Standard Deviation Calculation

The system dynamically calculates measurement trust (standard deviation) based on:
- **Tag count**: More tags = lower std dev (more trust)
- **Average distance**: Farther tags = higher std dev (less trust)
- **Robot speed**: Faster motion = higher std dev (less trust)
- **Single tag penalty**: Single tag MegaTag1 gets extra std dev

```java
// MegaTag1 std dev calculation
transStdDev = 0.3                                          // Initial
            - min(tagCount, 4) * 0.15                      // Tag count reward
            + avgTagDist * 0.1                             // Distance punishment
            + robotSpeed * 0.15                            // Speed punishment
            + (singleTag ? 0.3 : 0)                        // Single tag punishment

// MegaTag2 std dev calculation  
transStdDev = 0.2                                          // Initial
            - (tagCount > 1 ? 0.05 : 0)                    // Multi-tag bonus
            + avgTagDist * 0.075                           // Distance punishment
            + robotSpeed * 0.25                            // Speed punishment
```