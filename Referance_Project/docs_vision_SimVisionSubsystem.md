# SimVisionSubsystem Class

## Overview

`SimVisionSubsystem` provides vision simulation support using PhotonVision's simulation framework. It enables testing vision-based localization in the WPILib simulator without real cameras.

## File Location

```
src/main/java/com/spartronics4915/frc2025/subsystems/vision/SimVisionSubsystem. java
```

## Class Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          SimVisionSubsystem                                  │
│              extends SubsystemBase                                           │
│              implements VisionDeviceSubsystem                                │
├──────────────────────────────────────────────────────────────────────────────┤
│  - swerveDrive: SwerveDrive                                                  │
│  - visionSim: VisionSystemSim                                                │
│  - camera: PhotonCamera                                                      │
│  - photonPoseEstimator: PhotonPoseEstimator                                  │
│  - reefPoseEst: Optional<EstimatedRobotPose>                                 │
│  - reefCameraEstimator: PhotonPoseEstimator                                  │
│  - fiducialIds: ArrayList<Integer>                                           │
├──────────────────────────────────────────────────────────────────────────────┤
│  + SimVisionSubsystem(swerveSubsystem:  SwerveSubsystem)                      │
│  + getVisibleTagIDs(): ArrayList<Integer>                                    │
│  + getBotPose2dFromReefCamera(): Optional<Pose2d>                            │
│  + simulationPeriodic(): void                                                │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Architecture

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                      SIMULATION VISION ARCHITECTURE                            │
└────────────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────────┐
    │                         SimVisionSubsystem                              │
    │  ┌──────────────────────────────────────────────────────────────────┐   │
    │  │                      VisionSystemSim                              │   │
    │  │  ┌──────────────────────────────────────────────────────────┐    │   │
    │  │  │              AprilTagFieldLayout                         │    │   │
    │  │  │  - All 2025 Reefscape AprilTag positions                 │    │   │
    │  │  └──────────────────────────────────────────────────────────┘    │   │
    │  │                            │                                      │   │
    │  │                            ▼                                      │   │
    │  │  ┌──────────────────────────────────────────────────────────┐    │   │
    │  │  │                   PhotonCameraSim                        │    │   │
    │  │  │  - Simulates camera view based on robot pose             │    │   │
    │  │  │  - Generates synthetic AprilTag detections               │    │   │
    │  │  └──────────────────────────────────────────────────────────┘    │   │
    │  └──────────────────────────────────────────────────────────────────┘   │
    │                              │                                          │
    │                              ▼                                          │
    │  ┌──────────────────────────────────────────────────────────────────┐   │
    │  │                    PhotonPoseEstimator                            │   │
    │  │  - Strategy:  MULTI_TAG_PNP_ON_COPROCESSOR                         │   │
    │  │  - Fallback: LOWEST_AMBIGUITY                                     │   │
    │  │  - Estimates robot pose from simulated detections                 │   │
    │  └──────────────────────────────────────────────────────────────────┘   │
    └─────────────────────────────────────────────────────────────────────────┘
```

## Camera Configuration

The simulated camera is configured with:

```java
// Camera position relative to robot center
Translation3d robotToCameraTrl = new Translation3d(-0.1, 0, 0.5);
// -0.1m backward from center
// 0m left/right (centered)
// 0.5m up from floor

// Camera orientation
Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
// Pitched 15 degrees up (looking forward and slightly up)
```

```
                    CAMERA MOUNTING (Side View)
                    
                         ┌───┐ Camera
                         │ / │ (pitched -15°)
                         └─┬─┘
                           │ 0.5m
         ────────────┬─────┼─────┬────────────
                     │     │     │
                   Robot Center (0,0,0)
                     │  ◄──┘
                     │  0.1m back
                     ▼
                   Rear of Robot
```

## Simulation Update Flow

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                      simulationPeriodic() FLOW                                 │
└────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────┐
│   simulationPeriodic() called   │
│   (every sim loop)              │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│  Get simulated robot pose       │
│  from SwerveDrive               │
│                                 │
│  simPose = swerveDrive          │
│    .getSimulationDriveTrainPose │
│    ().get()                     │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│  Update vision simulation       │
│                                 │
│  visionSim.update(simPose)      │
│                                 │
│  - Calculates which tags are    │
│    visible from current pose    │
│  - Generates synthetic camera   │
│    results                      │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│  Process camera results         │
│                                 │
│  res = camera.getAllUnread-     │
│    Results()                    │
│                                 │
│  for (change : res) {           │
│    reefPoseEst = estimator      │
│      .update(change)            │
│  }                              │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│  Extract visible tag IDs        │
│                                 │
│  for (target : targetList) {    │
│    fiducialIds. add(             │
│      target.fiducialId)         │
│  }                              │
└─────────────────────────────────┘
```

## Pose Estimation Strategy

| Strategy | Description |
|----------|-------------|
| `MULTI_TAG_PNP_ON_COPROCESSOR` | Primary:  Uses PnP algorithm with multiple tags |
| `LOWEST_AMBIGUITY` | Fallback:  Uses single tag with lowest ambiguity |

## Key Methods

### `getBotPose2dFromReefCamera()`

Returns the estimated 2D robot pose from the simulated camera. 

```java
public Optional<Pose2d> getBotPose2dFromReefCamera() {
    if(reefPoseEst.isEmpty()) {
        return Optional.empty();
    }
    EstimatedRobotPose estimatedPose = reefPoseEst.get();
    Pose2d result = estimatedPose.estimatedPose. toPose2d();
    return Optional. of(result);
}
```

### `getVisibleTagIDs()`

Returns list of AprilTag IDs currently "visible" in simulation.

```java
public ArrayList<Integer> getVisibleTagIDs() {
    return fiducialIds;
}
```

## Usage

The SimVisionSubsystem is automatically used when running in simulation mode: 

```java
// In RobotContainer
VisionDeviceSubsystem visionSubsystem;
if (Robot.isSimulation()) {
    visionSubsystem = new SimVisionSubsystem(swerveSubsystem);
} else {
    visionSubsystem = new LimelightVisionSubsystem(swerveSubsystem, fieldLayout);
}
```

## Dependencies

| Library | Purpose |
|---------|---------|
| PhotonVision | Vision simulation framework |
| `VisionSystemSim` | Manages simulated cameras and field |
| `PhotonCameraSim` | Simulates individual camera |
| `PhotonPoseEstimator` | Calculates pose from detections |