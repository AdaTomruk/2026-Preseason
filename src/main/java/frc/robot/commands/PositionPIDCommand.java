package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Elastic;
// Assuming these static imports exist based on your provided code
import static edu.wpi.first.units.Units.*; 
import static frc.robot.Constants.DriveConstants.*; // Placeholder for your tolerance constants

public class PositionPIDCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d goalPose;
    
    // PathPlanner Controller
    private PPHolonomicDriveController mDriveController = DriveConstants.AutoConstants.kAutoAlignPIDController;

    // CTRE Swerve Requests
    private final SwerveRequest.ApplyChassisSpeeds driveRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final Timer timer = new Timer();
    private final Debouncer endTriggerDebouncer = new Debouncer(DriveConstants.AutoConstants.kEndTriggerDebounce.in(Seconds));

    // Logging
    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();

    private PositionPIDCommand(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;
        // IMPORTANT: Must require the subsystem
        addRequirements(drivetrain);
    }

    public static Command generateCommand(CommandSwerveDrivetrain drivetrain, Pose2d goalPose, Time timeout) {
        return new PositionPIDCommand(drivetrain, goalPose)
            .withTimeout(timeout)
            .finallyDo(() -> {
                // Stop the robot and lock modules (X-Stance)
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;
        // Current pose from CTRE Swerve State
        Pose2d currentPose = drivetrain.getState().Pose;

        // Calculate Speeds
        var speeds = mDriveController.calculateRobotRelativeSpeeds(currentPose, goalState);

        // Apply to drivetrain using CTRE Request
        drivetrain.setControl(driveRequest.withSpeeds(speeds));

        // Logging
        xErrLogger.accept(currentPose.getX() - goalPose.getX());
        yErrLogger.accept(currentPose.getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d diff = currentPose.relativeTo(goalPose);
        
        // Calculate velocity magnitude from X and Y speeds
        double currentVelocity = Math.hypot(
            drivetrain.getState().Speeds.vxMetersPerSecond, 
            drivetrain.getState().Speeds.vyMetersPerSecond
        );

        System.out.println("Adjustments to alignment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
            + "\nVelocity value: " + currentVelocity + "m/s"
        );

        if (interrupted) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("Auto Align Interrupted")
                .withDescription("The auto align command was interrupted " + Math.round(Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) * 1000.0) / 1000.0 + " cm from its target")
                .withDisplaySeconds(4.0)
            );
        }
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d diff = currentPose.relativeTo(goalPose);

        var rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            Constants.DriveConstants.AutoConstants.kRotationTolerance.getRotations(), 
            0.0, 
            1.0
        );

        var position = diff.getTranslation().getNorm() < Constants.DriveConstants.AutoConstants.kPositionTolerance.in(Meters);

        // Calculate velocity magnitude
        double currentSpeed = Math.hypot(
            drivetrain.getState().Speeds.vxMetersPerSecond, 
            drivetrain.getState().Speeds.vyMetersPerSecond
        );

        var speed = currentSpeed < Constants.DriveConstants.AutoConstants.kSpeedTolerance.in(MetersPerSecond);

        return endTriggerDebouncer.calculate(
            rotation && position && speed
        );
    }
}