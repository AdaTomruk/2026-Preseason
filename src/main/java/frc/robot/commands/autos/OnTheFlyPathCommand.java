package frc.robot.commands.autos;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Generates and follows a PathPlanner path "on the fly" from the current robot position to a target pose.
 * This command dynamically creates a smooth path using PathPlanner's path generation capabilities,
 * then follows it using PathPlanner's path following controller.
 */
public class OnTheFlyPathCommand {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final PathConstraints pathConstraints;
    private final double finalAdjustTimeout;

    /**
     * Creates a new OnTheFlyPathCommand generator.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param pathConstraints The constraints for the path (max velocity, max acceleration, etc.)
     * @param finalAdjustTimeout Timeout for final position adjustment (seconds)
     */
    public OnTheFlyPathCommand(
            CommandSwerveDrivetrain drivetrain,
            PathConstraints pathConstraints,
            double finalAdjustTimeout) {
        this.drivetrain = drivetrain;
        this.pathConstraints = pathConstraints;
        this.finalAdjustTimeout = finalAdjustTimeout;
    }

    /**
     * Generates a command that drives the robot to the target pose.
     * This command is deferred so the path is generated when the command is scheduled,
     * not when it's created.
     * 
     * @param targetPose The target pose to drive to (field-relative)
     * @return A command that drives to the target pose
     */
    public Command generatePathCommand(Pose2d targetPose) {
        return Commands.defer(() -> {
            Pose2d currentPose = drivetrain.getState().Pose;
            ChassisSpeeds currentSpeeds = drivetrain.getState().Speeds;
            
            // Calculate starting velocity heading
            Rotation2d velocityHeading = getPathVelocityHeading(currentSpeeds, targetPose, currentPose);
            
            // Create waypoints from current pose to target pose
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(), velocityHeading),
                targetPose
            );
            
            // Check if we're already very close to the target
            if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
                // If we're already at the target, just use PID to fine-tune position
                return Commands.sequence(
                    Commands.print("Start position PID loop (already near target)"),
                    PositionPIDCommand.generateCommand(drivetrain, targetPose, finalAdjustTimeout),
                    Commands.print("End position PID loop")
                );
            }
            
            // Calculate starting velocity magnitude
            double startingVelMagnitude = getVelocityMagnitude(currentSpeeds);
            // Ensure minimum velocity for smoother path generation
            startingVelMagnitude = Math.max(startingVelMagnitude, 0.1);
            
            // Create the path with ideal starting state and goal end state
            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                pathConstraints,
                new IdealStartingState(
                    MetersPerSecond.of(startingVelMagnitude),
                    currentPose.getRotation()
                ), 
                new GoalEndState(0.0, targetPose.getRotation())
            );
            
            // Prevent the path from being flipped based on alliance
            path.preventFlipping = true;
            
            // Follow the path, then use PID for final adjustment
            return AutoBuilder.followPath(path).andThen(
                Commands.print("Start position PID loop (final adjustment)"),
                PositionPIDCommand.generateCommand(drivetrain, targetPose, finalAdjustTimeout),
                Commands.print("End position PID loop")
            ).finallyDo((interrupted) -> {
                if (interrupted) {
                    // Stop the drivetrain if interrupted
                    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
                    com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds stopRequest = 
                        new com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds();
                    drivetrain.setControl(stopRequest.withSpeeds(speeds));
                }
            });
        }, Set.of(drivetrain));
    }

    /**
     * Calculates the heading for path generation based on current velocity.
     * If moving slowly, aims directly at target. If moving faster, uses velocity direction.
     * 
     * @param chassisSpeeds Current robot speeds (field-relative)
     * @param target Target pose
     * @param currentPose Current robot pose
     * @return Heading to use for path generation
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds chassisSpeeds, Pose2d target, Pose2d currentPose) {
        double velocityMag = getVelocityMagnitude(chassisSpeeds);
        
        if (velocityMag < 0.25) {
            // If moving slowly, aim straight at target
            Translation2d diff = target.getTranslation().minus(currentPose.getTranslation());
            // If already at target, use target rotation
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }
        
        // If moving faster, use velocity direction for smoother path
        return new Rotation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    /**
     * Calculates the magnitude of the robot's velocity.
     * 
     * @param chassisSpeeds Current robot speeds
     * @return Velocity magnitude in meters per second
     */
    private double getVelocityMagnitude(ChassisSpeeds chassisSpeeds) {
        return new Translation2d(
            chassisSpeeds.vxMetersPerSecond, 
            chassisSpeeds.vyMetersPerSecond
        ).getNorm();
    }
}
