package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPointCommand {
    
    private final CommandSwerveDrivetrain drivetrain;
    public static boolean isPIDLoopRunning = false;

    // -- PUBLISHERS FOR ADVANTAGESCOPE --
    // Logs the single final target pose (visualize as "2D Robot" or "Ghost")
    private final StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
        .getTable("logging").getStructTopic("DriveToPoint/TargetPose", Pose2d.struct).publish();

    // Logs the generated waypoints (visualize as "2D Poses" or "Trajectory")
    private final StructArrayPublisher<Pose2d> pathPublisher = NetworkTableInstance.getDefault()
        .getTable("logging").getStructArrayTopic("DriveToPoint/GeneratedPath", Pose2d.struct).publish();

    public DriveToPointCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    private PathConstraints pathConstraints = Constants.DriveConstants.AutoConstants.kAutoPathConstraints;

    public Command generateCommand(final Pose2d targetPose) {
        return Commands.defer(() -> {
            // 1. Publish the target for visualization
            targetPosePublisher.accept(targetPose);

            return getPathFromWaypoint(targetPose);
        }, Set.of(drivetrain));
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        Pose2d offsetWaypoint = new Pose2d(
            waypoint.getTranslation().plus(new Translation2d(
                Constants.DriveConstants.autoApproachOffset.in(Meters), 
                waypoint.getRotation().rotateBy(Rotation2d.k180deg)
            )),
            waypoint.getRotation()
        );

        Pose2d currentPose = drivetrain.getState().Pose;
        ChassisSpeeds currentSpeeds = drivetrain.getFieldVelocity();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentPose.getTranslation(), getPathVelocityHeading(currentSpeeds, offsetWaypoint)),
            offsetWaypoint
        );

        // 2. Publish the simple path points for visualization
        // (Converts PathPlanner waypoints to Pose2d for logging)
        pathPublisher.set(waypoints.stream()
            .map(p -> new Pose2d(p.anchor(), new Rotation2d())) 
            .toArray(Pose2d[]::new)
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return Commands.sequence(
                Commands.print("Start position PID loop"),
                PositionPIDCommand.generateCommand(drivetrain, waypoint, Constants.DriveConstants.AutoConstants.kAutoAlignAdjustTimeout),
                Commands.print("End position PID loop")
            );
        }

        LinearVelocity startingVel = getVelocityMagnitude(currentSpeeds);
        if (DriverStation.isAutonomous()) {
            startingVel = MetersPerSecond.of(Math.max(startingVel.in(MetersPerSecond), 0.1));
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            DriverStation.isAutonomous() ? pathConstraints : Constants.DriveConstants.AutoConstants.kTeleopPathConstraints,
            new IdealStartingState(startingVel, currentPose.getRotation()), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path)
            .andThen(
                Commands.print("Start position PID loop"),
                PositionPIDCommand.generateCommand(drivetrain, waypoint, (
                    DriverStation.isAutonomous() 
                        ? Constants.DriveConstants.AutoConstants.kAutoAlignAdjustTimeout 
                        : Constants.DriveConstants.AutoConstants.kTeleopAlignAdjustTimeout
                ))
                .beforeStarting(Commands.runOnce(() -> { isPIDLoopRunning = true; }))
                .finallyDo(() -> { isPIDLoopRunning = false; }),
                Commands.print("End position PID loop")
            )
            .finallyDo((interrupted) -> {
                if (interrupted) {
                    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
                }
            });
    }

    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        Pose2d currentPose = drivetrain.getState().Pose;
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.getTranslation().minus(currentPose.getTranslation());
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(Math.hypot(cs.vxMetersPerSecond, cs.vyMetersPerSecond));
    }
}