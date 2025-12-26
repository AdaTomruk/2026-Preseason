package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Factory class for creating autonomous commands using on-the-fly path generation.
 * These can be added to the auto chooser in RobotContainer.
 */
public class Autos {
    
    /**
     * Creates a simple test auto that drives in a square pattern.
     * This demonstrates on-the-fly path generation with multiple waypoints.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @return Command that drives in a square pattern
     */
    public static Command squarePattern(CommandSwerveDrivetrain drivetrain) {
        OnTheFlyPathCommand pathGenerator = new OnTheFlyPathCommand(
            drivetrain,
            AutoConstants.kAutoPathConstraints,
            AutoConstants.kAutoAlignAdjustTimeout
        );
        
        // Define a square pattern (assuming robot starts at origin)
        Pose2d point1 = new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0));
        Pose2d point2 = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90));
        Pose2d point3 = new Pose2d(0.0, 2.0, Rotation2d.fromDegrees(180));
        Pose2d point4 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(270));
        
        return Commands.sequence(
            Commands.print("Starting square pattern auto"),
            pathGenerator.generatePathCommand(point1),
            Commands.print("Reached point 1"),
            pathGenerator.generatePathCommand(point2),
            Commands.print("Reached point 2"),
            pathGenerator.generatePathCommand(point3),
            Commands.print("Reached point 3"),
            pathGenerator.generatePathCommand(point4),
            Commands.print("Completed square pattern")
        );
    }
    
    /**
     * Creates a simple drive forward auto.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @return Command that drives forward 2 meters
     */
    public static Command driveForward(CommandSwerveDrivetrain drivetrain) {
        OnTheFlyPathCommand pathGenerator = new OnTheFlyPathCommand(
            drivetrain,
            AutoConstants.kAutoPathConstraints,
            AutoConstants.kAutoAlignAdjustTimeout
        );
        
        // Use a deferred command to calculate the target pose when the command is scheduled
        return Commands.sequence(
            Commands.print("Driving forward 2 meters"),
            Commands.defer(() -> {
                Pose2d currentPose = drivetrain.getState().Pose;
                Pose2d targetPose = new Pose2d(
                    currentPose.getX() + 2.0,
                    currentPose.getY(),
                    currentPose.getRotation()
                );
                return pathGenerator.generatePathCommand(targetPose);
            }, java.util.Set.of(drivetrain)),
            Commands.print("Finished driving forward")
        );
    }
    
    /**
     * Creates an auto that demonstrates all three types of path generation.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @return Command demonstrating different path generation methods
     */
    public static Command demonstrationAuto(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            Commands.print("=== On-The-Fly Path Generation Demo ==="),
            
            // Method 1: Simple point-to-point with trapezoidal profile
            Commands.print("Method 1: DriveToPointCommand"),
            AutoCommandExamples.exampleDriveToPoint(drivetrain),
            Commands.waitSeconds(0.5),
            
            // Method 2: Precise pose with PID
            Commands.print("Method 2: PositionPIDCommand"),
            AutoCommandExamples.exampleDriveToPose(drivetrain),
            Commands.waitSeconds(0.5),
            
            // Method 3: Full PathPlanner path generation
            Commands.print("Method 3: OnTheFlyPathCommand"),
            AutoCommandExamples.exampleOnTheFlyPath(drivetrain),
            
            Commands.print("=== Demo Complete ===")
        );
    }
}
