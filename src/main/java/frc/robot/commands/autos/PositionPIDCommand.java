package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that uses PID control to precisely position the robot at a target pose.
 * This provides fine-tuned control for the final positioning after path following.
 */
public class PositionPIDCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    private final double timeout;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.ApplyFieldSpeeds fieldSpeedsRequest;
    
    private final Timer timer;
    private final double positionTolerance;
    private final double angleTolerance;

    /**
     * Creates a new PositionPIDCommand.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param targetPose The target pose to reach
     * @param timeout Maximum time to run the command (seconds)
     */
    public PositionPIDCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose, double timeout) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.timeout = timeout;
        
        // PID constants - these may need tuning for your robot
        this.xController = new PIDController(2.0, 0, 0);
        this.yController = new PIDController(2.0, 0, 0);
        this.thetaController = new PIDController(3.0, 0, 0);
        
        // Enable continuous input for theta controller (wraps around at ±180°)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        this.fieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
        this.timer = new Timer();
        
        // Tolerances for considering the target reached
        this.positionTolerance = 0.05; // 5 cm
        this.angleTolerance = Math.toRadians(2); // 2 degrees
        
        addRequirements(drivetrain);
    }

    /**
     * Factory method to generate a PositionPIDCommand.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param targetPose The target pose to reach
     * @param timeout Maximum time to run the command (seconds)
     * @return A new PositionPIDCommand
     */
    public static Command generateCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose, double timeout) {
        return new PositionPIDCommand(drivetrain, targetPose, timeout);
    }

    @Override
    public void initialize() {
        timer.restart();
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Calculate PID outputs for x, y, and theta
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(
            currentPose.getRotation().getRadians(), 
            targetPose.getRotation().getRadians()
        );
        
        // Create chassis speeds and apply to drivetrain
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        drivetrain.setControl(fieldSpeedsRequest.withSpeeds(speeds));
    }

    @Override
    public boolean isFinished() {
        // Check if we've reached the target or timed out
        if (timer.hasElapsed(timeout)) {
            return true;
        }
        
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d positionError = targetPose.getTranslation().minus(currentPose.getTranslation());
        double distanceError = positionError.getNorm();
        
        double angleError = Math.abs(
            targetPose.getRotation().minus(currentPose.getRotation()).getRadians()
        );
        
        return distanceError < positionTolerance && angleError < angleTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain and apply X-brake
        SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        drivetrain.setControl(brake);
    }
}
