package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Command that uses a trapezoidal motion profile to drive to a point in a smooth fashion.
 * This generates a path "on the fly" without using PathPlanner's path files.
 */
public class DriveToPointCommand extends Command {

    private final Translation2d targetPoint;
    private final TrapezoidProfile translationProfile;
    private final CommandSwerveDrivetrain drivetrain;
    private final double closeEnoughThreshold;
    private double velocitySetpoint;
    private boolean nearGoal; 
    private final double minimumSpeed;
    private final SwerveRequest.ApplyFieldSpeeds fieldSpeedsRequest;

    /**
     * Creates a new DriveToPointCommand.
     * 
     * @param targetPoint The target point to drive to (field-relative)
     * @param translationConstraints The constraints for the trapezoidal profile (max velocity and acceleration)
     * @param closeEnoughThreshold The distance threshold to consider the target reached (meters)
     * @param minimumSpeed The minimum speed to maintain while driving (meters/second)
     * @param drivetrain The swerve drivetrain subsystem
     */
    public DriveToPointCommand(Translation2d targetPoint, Constraints translationConstraints,
            double closeEnoughThreshold, double minimumSpeed,
            CommandSwerveDrivetrain drivetrain) {
        this.targetPoint = targetPoint;
        this.translationProfile = new TrapezoidProfile(translationConstraints);
        this.drivetrain = drivetrain;
        this.closeEnoughThreshold = closeEnoughThreshold;
        this.minimumSpeed = minimumSpeed;
        this.fieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();

        velocitySetpoint = 0;
        nearGoal = false;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        velocitySetpoint = 0;
        nearGoal = false;
    }

    @Override
    public void execute() {
        Pose2d currPose = drivetrain.getState().Pose;
        Translation2d translationToTarget = targetPoint.minus(currPose.getTranslation());
        double distanceRemaining = translationToTarget.getNorm();

        if((distanceRemaining < closeEnoughThreshold) || nearGoal) {
            nearGoal = true;
            return;
        }

        final double dT = 1 / 50.;

        State goalState = new State(distanceRemaining, 0);
        State currentState = new State(0, velocitySetpoint);
        State outputState = translationProfile.calculate(dT, currentState, goalState);

        double driveVelocity = outputState.velocity;

        if(driveVelocity < minimumSpeed) {
            driveVelocity = minimumSpeed;
        }

        Translation2d driveTranslation = translationToTarget.div(translationToTarget.getNorm())
                .times(driveVelocity);

        ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(driveTranslation.getX(),
                driveTranslation.getY(), 0);

        drivetrain.setControl(fieldSpeedsRequest.withSpeeds(newChassisSpeeds));
        velocitySetpoint = outputState.velocity;
    }

    @Override
    public boolean isFinished() {
        return nearGoal;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        drivetrain.setControl(fieldSpeedsRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
