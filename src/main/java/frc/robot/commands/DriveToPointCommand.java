package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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

        // 2. Publish the offset waypoint for visualization
        pathPublisher.set(new Pose2d[] { currentPose, offsetWaypoint });

        // Check if robot is already at target
        if (currentPose.getTranslation().getDistance(offsetWaypoint.getTranslation()) < 0.01) {
            return Commands.sequence(
                Commands.print("Start position PID loop"),
                PositionPIDCommand.generateCommand(drivetrain, waypoint, Constants.DriveConstants.AutoConstants.kAutoAlignAdjustTimeout),
                Commands.print("End position PID loop")
            );
        }

        // Use pathfinding to navigate around obstacles
        PathConstraints constraints = DriverStation.isAutonomous() 
            ? Constants.DriveConstants.AutoConstants.kAutoPathConstraints 
            : Constants.DriveConstants.AutoConstants.kTeleopPathConstraints;

        return AutoBuilder.pathfindToPose(
            offsetWaypoint,
            constraints,
            0.0 // Goal end velocity
        )
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
}