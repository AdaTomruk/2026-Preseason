# 2026 Preseason Robot Code

This repository contains the robot code for the 2026 preseason robot, using the WPILib command-based framework and CTRE Phoenix 6 swerve drive.

## Features

### Vision System
The robot includes a vision system that uses different implementations for simulation and real robot:

- **Real Robot**: Uses Limelight cameras via `LimelightVisionSubsystem`
- **Simulation**: Uses PhotonVision simulation via `SimVisionSubsystem`

The vision subsystem automatically switches between these implementations based on whether the robot is running in simulation or on real hardware.

### Simulation Support
PhotonVision simulation is now fully supported, providing:
- AprilTag detection and tracking in simulation
- Vision-based pose estimation using PhotonPoseEstimator
- Automatic integration with the swerve drivetrain odometry
- Realistic camera simulation with configurable properties

## Building and Running

### Prerequisites
- WPILib 2025.3.2 or later
- Java 17

### Building
```bash
./gradlew build
```

### Running in Simulation
```bash
./gradlew simulateJava
```

When running in simulation, the `SimVisionSubsystem` will automatically be used, providing simulated AprilTag vision data.

### Deploying to Robot
```bash
./gradlew deploy
```

When deployed to a real robot, the `LimelightVisionSubsystem` will be used instead.

## Project Structure

- `src/main/java/frc/robot/` - Main robot code
  - `subsystems/` - Robot subsystems
    - `vision/` - Vision subsystems (Limelight and PhotonVision simulation)
    - `CommandSwerveDrivetrain.java` - Swerve drivetrain subsystem
  - `RobotContainer.java` - Robot hardware and command configuration
  - `Robot.java` - Main robot class

## Vendor Dependencies

This project uses the following vendor libraries:
- CTRE Phoenix 6 (Swerve drive)
- PathPlanner (Autonomous path following)
- PhotonLib (Vision simulation)

## Reference

This simulation implementation is based on the [Spartronics 2025 Reefscape](https://github.com/Spartronics4915/2025-Reefscape) reference project.
