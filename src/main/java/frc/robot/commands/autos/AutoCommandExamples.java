package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Example usage patterns for the on-the-fly path generation commands.
 * This class demonstrates how to use DriveToPointCommand, PositionPIDCommand, and OnTheFlyPathCommand
 * in your robot code.
 */
public class AutoCommandExamples {
    
    /**
     * Example 1: Drive to a specific point using trapezoidal motion profile.
     * This is simpler but less sophisticated than using PathPlanner paths.
     * 
     * @param drivetrain The swerve drivetrain
     * @return Command that drives to the target point
     */
    public static Command exampleDriveToPoint(CommandSwerveDrivetrain drivetrain) {
        // Define a target point on the field (in meters, field-relative)
        Translation2d targetPoint = new Translation2d(5.0, 3.0);
        
        return new DriveToPointCommand(
            targetPoint,
            AutoConstants.kDriveToPointConstraints,
            AutoConstants.kDriveToPointTolerance,
            AutoConstants.kMinimumDriveSpeed,
            drivetrain
        );
    }
    
    /**
     * Example 2: Drive to a specific pose (position + rotation) using PID control.
     * This provides precise positioning but may not be as smooth as path following.
     * 
     * @param drivetrain The swerve drivetrain
     * @return Command that drives to the target pose
     */
    public static Command exampleDriveToPose(CommandSwerveDrivetrain drivetrain) {
        // Define a target pose on the field (x, y in meters, rotation)
        Pose2d targetPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
        
        return PositionPIDCommand.generateCommand(
            drivetrain,
            targetPose,
            AutoConstants.kAutoAlignAdjustTimeout
        );
    }
    
    /**
     * Example 3: Generate and follow a PathPlanner path on the fly.
     * This is the most sophisticated approach, providing smooth acceleration and deceleration.
     * 
     * @param drivetrain The swerve drivetrain
     * @return Command that generates and follows a path to the target pose
     */
    public static Command exampleOnTheFlyPath(CommandSwerveDrivetrain drivetrain) {
        // Create the path generator
        OnTheFlyPathCommand pathGenerator = new OnTheFlyPathCommand(
            drivetrain,
            AutoConstants.kAutoPathConstraints,
            AutoConstants.kAutoAlignAdjustTimeout
        );
        
        // Define a target pose
        Pose2d targetPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
        
        // Generate the command (deferred so path is created when scheduled)
        return pathGenerator.generatePathCommand(targetPose);
    }
    
    /**
     * Example 4: Create a sequence that drives to multiple waypoints.
     * This demonstrates chaining multiple on-the-fly paths together.
     * 
     * @param drivetrain The swerve drivetrain
     * @return Command sequence that drives through multiple waypoints
     */
    public static Command exampleMultiWaypointPath(CommandSwerveDrivetrain drivetrain) {
        OnTheFlyPathCommand pathGenerator = new OnTheFlyPathCommand(
            drivetrain,
            AutoConstants.kAutoPathConstraints,
            AutoConstants.kAutoAlignAdjustTimeout
        );
        
        // Define waypoints
        Pose2d waypoint1 = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0));
        Pose2d waypoint2 = new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(90));
        Pose2d waypoint3 = new Pose2d(7.0, 3.0, Rotation2d.fromDegrees(180));
        
        // Create a sequence of path commands
        return pathGenerator.generatePathCommand(waypoint1)
            .andThen(pathGenerator.generatePathCommand(waypoint2))
            .andThen(pathGenerator.generatePathCommand(waypoint3));
    }
    
    /**
     * Example 5: Drive to a pose during teleop with slower constraints.
     * This uses different path constraints suitable for teleop operation.
     * 
     * @param drivetrain The swerve drivetrain
     * @param targetPose The target pose to drive to
     * @return Command that drives to the target pose with teleop constraints
     */
    public static Command teleopDriveToTarget(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        OnTheFlyPathCommand pathGenerator = new OnTheFlyPathCommand(
            drivetrain,
            AutoConstants.kTeleopPathConstraints,  // Use teleop constraints (slower)
            AutoConstants.kTeleopAlignAdjustTimeout
        );
        
        return pathGenerator.generatePathCommand(targetPose);
    }
}
