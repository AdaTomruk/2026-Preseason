package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

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
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPointCommand {
    
    private final CommandSwerveDrivetrain drivetrain;
    
    public static void warmup(){
        System.out.println("Drive To Point Command is Warmed Up");
    }

    public static boolean isPIDLoopRunning = false;

    public DriveToPointCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("targeted position", Pose2d.struct).publish();

    private PathConstraints pathConstraints = Constants.DriveConstants.AutoConstants.kAutoPathConstraints;

    public Command generateCommand(final Pose2d targetPose){
        return Commands.defer(() ->{
            return new getPathFromWaypoint(getWatpointFromPose2D(targetPose));
        })
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        Pose2d offsetWaypoint = new Pose2d(
            waypoint.getTranslation().plus(new Translation2d(Constants.DriveConstants.autoApproachOffset.in(Meters), waypoint.getRotation().rotateBy(Rotation2d.k180deg))),
            waypoint.getRotation()
        );

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(drivetrain.getState().Pose.getTranslation(), getPathVelocityHeading(drivetrain.getFieldVelocity(), offsetWaypoint)),
            offsetWaypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return 
            Commands.sequence(
                Commands.print("start position PID loop"),
                PositionPIDCommand.generateCommand(drivetrain, waypoint, Constants.DriveConstants.AutoConstants.kAutoAlignAdjustTimeout),
                Commands.print("end position PID loop")
            );
        }
        var startingVel = getVelocityMagnitude(drivetrain.getFieldVelocity());

        if (DriverStation.isAutonomous()) {
            startingVel = MetersPerSecond.of(
                Math.max(startingVel.in(MetersPerSecond), 0.1)
            );
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            DriverStation.isAutonomous() ? pathConstraints :Constants.DriveConstants.AutoConstants.kTeleopPathConstraints,
            new IdealStartingState(
                startingVel,
                drivetrain.getState().RawHeading
            ), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return (AutoBuilder.followPath(path).andThen(
            Commands.print("start position PID loop"),
            PositionPIDCommand.generateCommand(drivetrain, waypoint, (
                DriverStation.isAutonomous() ? Constants.DriveConstants.AutoConstants.kAutoAlignAdjustTimeout : Constants.DriveConstants.AutoConstants.kTeleopAlignAdjustTimeout
            ))
                .beforeStarting(Commands.runOnce(() -> {isPIDLoopRunning = true;}))
                .finallyDo(() -> {isPIDLoopRunning = false;}),
            Commands.print("end position PID loop")
        )).finallyDo((interrupt) -> {
            if (interrupt) { //if this is false then the position pid would've X braked & called the same method
                drivetrain.applyRequest(null);
            }
        });        

    }

    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            System.out.println("approach: straight line");
            var diff = target.getTranslation().minus(drivetrain.getState().Pose.getTranslation());
            System.out.println("diff calc: \nx: " + diff.getX() + "\ny: " + diff.getY() + "\nDoT: " + diff.getAngle().getDegrees());
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }

        System.out.println("approach: compensating for velocity");

        var rotation = new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
        
        System.out.println("velocity calc: \nx: " + cs.vxMetersPerSecond + "\ny: " + cs.vyMetersPerSecond + "\nDoT: " + rotation);

        return rotation;
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }


    private Pose2d getWatpointFromPose2D(Pose2d pose) {
        return new Pose2d();
    }

}
