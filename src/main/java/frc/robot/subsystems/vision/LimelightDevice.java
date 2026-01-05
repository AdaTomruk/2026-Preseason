package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.PoseEstimationMethod;
import frc.robot.Constants.VisionConstants.StdDevConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Structures.LimelightConstants;
import frc.robot.util.Structures.VisionMeasurement;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class LimelightDevice extends SubsystemBase {
    
    private final String name;
    private final VisionConstants.LimelightModel model;
    private final int id;
    private final VisionConstants.LimelightRole role;
    private int[] tagFilter = new int[]{};

    public LimelightDevice(LimelightConstants constants) {
        this.name = "limelight-" + constants.name();
        this.model = constants.model();
        this.id = constants.id();
        this.role = constants.role();
    }

    public Void setTagFilter(Optional <DriverStation.Alliance> alliance) {
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                this.tagFilter = VisionConstants.AprilTagRegions.RED_ALLIANCE;
            } else if (alliance.get() == DriverStation.Alliance.Blue) {
                this.tagFilter = VisionConstants.AprilTagRegions.BLUE_ALLIANCE;
            }
        } else {
            this.tagFilter = new int[]{};
        }
        // Assume LimelightHelpers.SetAprilTagFilter is a method to set the tag filter on the Limelight
        LimelightHelpers.SetFiducialIDFiltersOverride(this.name, this.tagFilter);
        return null;
    }

    public VisionConstants.LimelightRole getRole() {
        return this.role;
    }

    public double getTx() {
        return LimelightHelpers.getTX(this.name);
    }
    public double getTy() {
        return LimelightHelpers.getTY(this.name);
    }
    public boolean getTv() {
        return LimelightHelpers.getTV(this.name);
    }

    public ArrayList<Integer> getVisibleTagIDs() {
        RawFiducial[] rawTags = LimelightHelpers.getRawFiducials(this.name);
        ArrayList<Integer> visibleTagIDs = new ArrayList<>();
        for (RawFiducial tag : rawTags) {
            visibleTagIDs.add(tag.id);
        }
        return visibleTagIDs;
    }

    public Optional<VisionMeasurement> getVisionMeasurement(CommandSwerveDrivetrain swerve){
        if(this.role == VisionConstants.LimelightRole.LOCALIZATION){
            PoseEstimationMethod method = PoseEstimationMethod.MEGATAG_2; //default to megatag 2
            final boolean BEFORE_MATCH = !Robot.AUTO_TIMER.hasElapsed(0.01) && !Robot.TELEOP_TIMER.hasElapsed(0.01);
            final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
            if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
            if (!BEFORE_MATCH && poseEstimate.avgTagDist > VisionConstants.kMaxDistance) return Optional.empty();
            final boolean twoOrMoreTags = poseEstimate.tagCount >= 2;
            final boolean closeEnough = poseEstimate.avgTagDist < VisionConstants.kMaxDistanceForMegaTag1;
            double robotSpeed = Math.sqrt(Math.pow(swerve.getState().Speeds.vxMetersPerSecond,2) + Math.pow(swerve.getState().Speeds.vyMetersPerSecond,2)); //Calculate robot speed using pythagorean theorem
            final boolean movingSlowEnough = robotSpeed < VisionConstants.kMaxSpeedForMegaTag1;
            final boolean CAN_GET_GOOD_HEADING = twoOrMoreTags && movingSlowEnough && closeEnough;
            if (BEFORE_MATCH && !CAN_GET_GOOD_HEADING) return Optional.empty(); //we only want the best for our inital pose
            if (CAN_GET_GOOD_HEADING || LimelightVisionSubsystem.getMegaTag1Override()) method = PoseEstimationMethod.MEGATAG_1;
            return getVisionMeasurement(swerve, method);

        } else {
            return Optional.empty();
        }
    }

    public Optional<VisionMeasurement> getVisionMeasurement(CommandSwerveDrivetrain swerve, PoseEstimationMethod method) {
        if(role != VisionConstants.LimelightRole.LOCALIZATION){
            return Optional.empty();
        }

        PoseEstimate poseEstimate;
        Optional<Matrix<N3, N1>> stdDevs;

        //important: use SetRobotOrientation to use megatag2
        LimelightHelpers.SetRobotOrientation(name, swerve.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        switch (method) {
            case MEGATAG_1:
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
                stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
                break;
            case MEGATAG_2:
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);      
                stdDevs = calculateStdDevsMegaTag2(poseEstimate, swerve);
                break;  
            default:
                System.out.println("Unknown pose estimation method provided: " + method);
                return Optional.empty();
        }

        if(stdDevs.isEmpty()){
            return Optional.empty();
        }
        else{
            return Optional.of(new VisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                stdDevs.get(),
                name,
                poseEstimate.tagCount,
                poseEstimate.avgTagDist,
                Math.sqrt(Math.pow(swerve.getState().Speeds.vxMetersPerSecond,2) + Math.pow(swerve.getState().Speeds.vyMetersPerSecond,2)),
                method
                ));
        }
    }
    
    
    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag1(PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve){
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();

        double transStdDev = StdDevConstants.MegaTag1.kInitialValue;

        if (poseEstimate.tagCount ==  1 && poseEstimate.rawFiducials.length == 1){
            RawFiducial singleTag = poseEstimate.rawFiducials[0];
            if (VisionConstants.kVisionDiagnostics) SmartDashboard.putNumber("VisionDiagnostics/" + name + "/single tag pose ambiguity", singleTag.ambiguity);
            if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
                return Optional.empty();
            }
            transStdDev += StdDevConstants.MegaTag1.kSingleTagPunishment;
        }
        transStdDev -= Math.min(poseEstimate.tagCount, 4) * StdDevConstants.MegaTag1.kTagCountReward;
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag1.kAverageDistancePunishment;
        transStdDev += Math.sqrt(Math.pow(swerve.getState().Speeds.vxMetersPerSecond,2) + Math.pow(swerve.getState().Speeds.vyMetersPerSecond,2)) * StdDevConstants.MegaTag1.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, 0.05); //make sure we aren't putting all our trust in vision

        double rotStdDev = 0.3;

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));

    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(LimelightHelpers.PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        
        if (Math.abs(Units.radiansToDegrees(swerve.getState().Speeds.omegaRadiansPerSecond)) > VisionConstants.kMaxAngularSpeed) 
            return Optional.empty(); //don't trust if turning too fast
        if (poseEstimate.avgTagDist > 8) return Optional.empty();
        
        double transStdDev = StdDevConstants.MegaTag2.kInitialValue;

        if (poseEstimate.tagCount > 1) transStdDev -= StdDevConstants.MegaTag2.kMultipleTagsBonus;
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag2.kAverageDistancePunishment;
        transStdDev += Math.sqrt(Math.pow(swerve.getState().Speeds.vxMetersPerSecond,2) + Math.pow(swerve.getState().Speeds.vyMetersPerSecond,2)) * StdDevConstants.MegaTag2.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, 0.05); //make sure we aren't putting all our trust in vision

        double rotStdDev = Double.MAX_VALUE; //never trust rotation under any circumstances

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    } 
    
    public Optional<Pose2d> getPose2d() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (pose.tagCount == 0) {
            return Optional.empty();
        }
        return Optional.of(pose.pose);
    }
    

}