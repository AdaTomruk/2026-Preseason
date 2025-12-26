# Robot Simulation Guide

This guide covers how to simulate the robot and use AdvantageScope for telemetry visualization.

## Running the Simulation

### Using VS Code
1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`)
2. Type "WPILib: Simulate Robot Code"
3. Select your robot project
4. Choose "Sim GUI" when prompted

### Using Gradle
```bash
./gradlew simulateJava
```

## Using AdvantageScope

### Installation
1. Download AdvantageScope from [GitHub Releases](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
2. Install the application for your platform

### Connecting to Simulation
1. Start your robot simulation
2. Open AdvantageScope
3. Go to `File` → `Connect to Simulator`
4. AdvantageScope will automatically connect to the running simulation

### Key Features

#### Viewing Telemetry
- **Odometry**: Navigate to the "Odometry" tab to view robot position and pose
- **Network Tables**: Browse all NetworkTables values in the left sidebar
- **Graphs**: Add line graphs to visualize numeric data over time

#### Swerve Drive Visualization
1. Select the "Swerve" or "Odometry" tab
2. Add the pose data from NetworkTables (typically under `/SmartDashboard/Field`)
3. View the robot's position and module states in real-time

#### Vision Simulation
- Vision targets and camera feeds appear in the simulation
- View camera poses and detected targets in AdvantageScope's 3D field view

## Simulation Features

### Physics Simulation
- Swerve drive kinematics and dynamics
- Realistic motor behavior
- Gyro simulation with drift modeling

### Vision Simulation
- Simulated vision targets on the field
- Camera pose estimation
- AprilTag detection

### Testing Autonomous
1. Start the simulation
2. Select an autonomous routine from SmartDashboard
3. Enable autonomous mode in the Sim GUI
4. Watch the robot execute the path in AdvantageScope

## Tips
- Use the Sim GUI to control robot state (Disabled/Autonomous/Teleop)
- Monitor motor currents and temperatures in AdvantageScope
- Record simulation data using AdvantageScope's logging feature
- Use keyboard shortcuts in Sim GUI for faster testing

## Troubleshooting

### Simulation won't start
- Ensure all dependencies are installed: `./gradlew build`
- Check for compilation errors in the console

### AdvantageScope won't connect
- Verify simulation is running
- Check that NetworkTables is publishing data
- Restart both simulation and AdvantageScope

### Physics behaves unexpectedly
- Verify TunerConstants are configured correctly
- Check that all motor controllers have proper IDs
- Review swerve module configurations
