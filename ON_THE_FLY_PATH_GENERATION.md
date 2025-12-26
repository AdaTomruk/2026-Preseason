# On-The-Fly Path Generation

This implementation provides basic on-the-fly path generation capabilities for the swerve drivetrain, allowing the robot to generate and follow paths dynamically during runtime without pre-defined PathPlanner path files.

## Overview

The implementation consists of three main commands that work together to provide different levels of path generation sophistication:

### 1. DriveToPointCommand
A simple command that uses trapezoidal motion profiles to drive to a target point.

**Features:**
- Uses trapezoidal acceleration/deceleration for smooth motion
- Drives to a translation target (position only, no rotation control)
- Configurable constraints (max velocity, max acceleration)
- Configurable tolerance and minimum speed

**Use Cases:**
- Simple point-to-point navigation
- When rotation doesn't matter
- Quick testing and prototyping

**Example:**
```java
Translation2d targetPoint = new Translation2d(5.0, 3.0);
Command driveCommand = new DriveToPointCommand(
    targetPoint,
    AutoConstants.kDriveToPointConstraints,
    AutoConstants.kDriveToPointTolerance,
    AutoConstants.kMinimumDriveSpeed,
    drivetrain
);
```

### 2. PositionPIDCommand
A PID-based command for precise positioning at a target pose (position + rotation).

**Features:**
- Independent PID controllers for X, Y, and rotation
- Continuous input handling for rotation (wraps at ±180°)
- Configurable timeout
- X-brake when finished for stability

**Use Cases:**
- Final positioning after path following
- Precise pose alignment
- When you need exact position and rotation

**Example:**
```java
Pose2d targetPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
Command pidCommand = PositionPIDCommand.generateCommand(
    drivetrain,
    targetPose,
    AutoConstants.kAutoAlignAdjustTimeout
);
```

### 3. OnTheFlyPathCommand
The most sophisticated approach - generates PathPlanner paths dynamically and follows them.

**Features:**
- Generates smooth paths using PathPlanner's path generation
- Considers current velocity for smooth transitions
- Automatically adjusts for moving vs. stationary starts
- Falls back to PID for very short distances
- Final PID adjustment for precise positioning
- Configurable path constraints
- Deferred execution (path generated when scheduled, not when created)

**Use Cases:**
- Smooth autonomous navigation
- Dynamic target following
- Complex path sequences
- When you want the benefits of PathPlanner without pre-defined paths

**Example:**
```java
OnTheFlyPathCommand pathGenerator = new OnTheFlyPathCommand(
    drivetrain,
    AutoConstants.kAutoPathConstraints,
    AutoConstants.kAutoAlignAdjustTimeout
);

Pose2d targetPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
Command pathCommand = pathGenerator.generatePathCommand(targetPose);
```

## Configuration

All configuration constants are defined in `Constants.java` under the `AutoConstants` class:

```java
// PathPlanner constraints for autonomous
public static final PathConstraints kAutoPathConstraints = new PathConstraints(
    MetersPerSecond.of(4.0),           // Max velocity
    MetersPerSecondPerSecond.of(3.0),  // Max acceleration
    RadiansPerSecond.of(Math.toRadians(540)),      // Max angular velocity
    RadiansPerSecondPerSecond.of(Math.toRadians(720)) // Max angular acceleration
);

// PathPlanner constraints for teleop (slower for safety)
public static final PathConstraints kTeleopPathConstraints = new PathConstraints(
    MetersPerSecond.of(3.0),           // Max velocity
    MetersPerSecondPerSecond.of(2.0),  // Max acceleration
    RadiansPerSecond.of(Math.toRadians(360)),      // Max angular velocity
    RadiansPerSecondPerSecond.of(Math.toRadians(540)) // Max angular acceleration
);

// Trapezoidal profile constraints for DriveToPointCommand
public static final TrapezoidProfile.Constraints kDriveToPointConstraints = 
    new TrapezoidProfile.Constraints(4.0, 3.0);

// Tolerances and timeouts
public static final double kDriveToPointTolerance = 0.1; // meters
public static final double kMinimumDriveSpeed = 0.3; // meters per second
public static final double kAutoAlignAdjustTimeout = 1.5; // seconds
public static final double kTeleopAlignAdjustTimeout = 2.0; // seconds
```

## Usage Examples

See `AutoCommandExamples.java` for detailed examples, including:

1. **Simple drive to point**
2. **Precise pose alignment**
3. **On-the-fly path generation**
4. **Multi-waypoint sequences**
5. **Teleop target following**

## Tuning

### PID Constants (PositionPIDCommand)
Located in `PositionPIDCommand.java`:
- `xController`: Default P=2.0, I=0, D=0
- `yController`: Default P=2.0, I=0, D=0
- `thetaController`: Default P=3.0, I=0, D=0

Adjust these based on your robot's characteristics. Higher P values = more aggressive response.

### Path Constraints
Adjust the constraints in `Constants.java` based on your robot's capabilities and desired behavior:
- Increase max velocity/acceleration for faster movement
- Decrease for smoother, more controlled movement
- Use different constraints for auto vs. teleop

### Tolerances
- `kDriveToPointTolerance`: How close to get to the target (meters)
- `positionTolerance` in PositionPIDCommand: Position precision (meters)
- `angleTolerance` in PositionPIDCommand: Rotation precision (radians)

## Integration with RobotContainer

To use these commands in your robot code, you can:

1. **In autonomous routines:**
```java
public Command getAutonomousCommand() {
    OnTheFlyPathCommand pathGen = new OnTheFlyPathCommand(
        drivetrain, 
        AutoConstants.kAutoPathConstraints,
        AutoConstants.kAutoAlignAdjustTimeout
    );
    
    return pathGen.generatePathCommand(targetPose);
}
```

2. **Bound to controller buttons:**
```java
joystick.x().onTrue(
    AutoCommandExamples.teleopDriveToTarget(drivetrain, targetPose)
);
```

3. **In command groups:**
```java
return Commands.sequence(
    pathGen.generatePathCommand(waypoint1),
    pathGen.generatePathCommand(waypoint2),
    Commands.print("Finished multi-waypoint path")
);
```

## Reference

This implementation is based on the Spartronics4915 2025-Reefscape robot code:
https://github.com/Spartronics4915/2025-Reefscape/tree/2c89b362809618022c2fc1f46a8c7a4198c68396

Key concepts adapted:
- Trapezoidal profile motion for smooth acceleration
- Dynamic PathPlanner path generation
- PID-based final positioning
- Velocity-aware path generation

## Benefits

1. **No pre-defined paths needed**: Generate paths dynamically based on runtime conditions
2. **Smooth motion**: Uses PathPlanner's sophisticated path generation
3. **Flexible**: Three different approaches for different use cases
4. **Easy to use**: Simple API with sensible defaults
5. **Velocity-aware**: Considers current robot velocity for smooth transitions
6. **Precise**: Final PID adjustment ensures accurate positioning
