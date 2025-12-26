// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.LimelightConstants;
import frc.robot.Constants.VisionConstants.LimelightModel;
import frc.robot.Constants.VisionConstants.LimelightRole;
import frc.robot.Constants.VisionConstants.PoseEstimationMethod;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.util.VisionMeasurement;

public class LimelightDevice {
    private final String name;
    private final LimelightModel model;
    private final int id;
    private final LimelightRole role;

    public LimelightDevice(LimelightConstants constants) {
        this.name = constants.name();
        this.model = constants.model();
        this.id = constants.id();
        this.role = constants.role();
    }

    public String getName() {
        return name;
    }

    public LimelightModel getModel() {
        return model;
    }

    public int getId() {
        return id;
    }

    public LimelightRole getRole() {
        return role;
    }

    public double getTx() {
        return LimelightHelpers.getTX(name);
    }

    public double getTy() {
        return LimelightHelpers.getTY(name);
    }

    public boolean getTv() {
        return LimelightHelpers.getTV(name);
    }

    public int[] getVisibleTagIDs() {
        return LimelightHelpers.getFiducialIDList(name);
    }

    /**
     * Get vision measurement from this Limelight device.
     * Intelligently selects between MegaTag1 and MegaTag2 based on conditions.
     * 
     * @param robotSpeed Current robot speed in meters per second
     * @return Optional VisionMeasurement if valid pose is available
     */
    public Optional<VisionMeasurement> getVisionMeasurement(double robotSpeed) {
        int[] tagIds = getVisibleTagIDs();
        if (tagIds.length == 0) {
            return Optional.empty();
        }

        // Determine which pose estimation method to use
        PoseEstimationMethod method = selectPoseEstimationMethod(tagIds.length, robotSpeed);
        PoseEstimate estimate;
        
        if (method == PoseEstimationMethod.MEGATAG_1) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag(name);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        }

        if (estimate == null || estimate.tagCount == 0) {
            return Optional.empty();
        }

        // Calculate average distance to tags
        double avgDistance = estimate.avgTagDist;
        
        // Filter out measurements that are too far
        if (avgDistance > VisionConstants.kMaxDistance) {
            return Optional.empty();
        }

        // Calculate standard deviations based on tag count, distance, and robot speed
        var stdDevs = calculateStdDevs(estimate.tagCount, avgDistance, robotSpeed);

        return Optional.of(new VisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            stdDevs,
            name,
            estimate.tagCount,
            avgDistance,
            robotSpeed,
            method.name()
        ));
    }

    /**
     * Select the appropriate pose estimation method based on conditions.
     * 
     * MegaTag1 is used when:
     * - 2+ tags are visible
     * - Robot is moving slowly
     * - Tags are close enough
     * 
     * Otherwise, MegaTag2 is used.
     */
    private PoseEstimationMethod selectPoseEstimationMethod(int tagCount, double robotSpeed) {
        PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        if (mt2Estimate == null) {
            return PoseEstimationMethod.MEGATAG_2;
        }

        double avgDistance = mt2Estimate.avgTagDist;
        
        boolean useMegaTag1 = tagCount >= 2 
            && robotSpeed <= VisionConstants.kMaxSpeedForMegaTag1
            && avgDistance <= VisionConstants.kMaxDistanceForMegaTag1;

        return useMegaTag1 ? PoseEstimationMethod.MEGATAG_1 : PoseEstimationMethod.MEGATAG_2;
    }

    /**
     * Calculate standard deviations for vision measurement.
     * Lower values = more trust in the measurement.
     * 
     * Standard deviations scale with:
     * - Fewer tags visible = higher std dev
     * - Greater distance = higher std dev
     * - Higher robot speed = higher std dev
     */
    private Matrix<N3, N1> calculateStdDevs(int tagCount, double avgDistance, double robotSpeed) {
        
        // Base standard deviations (x, y, theta)
        double xyStdDev = 0.5;
        double thetaStdDev = 0.5;

        // Scale based on tag count (more tags = more confidence)
        double tagCountFactor = 1.0;
        if (tagCount >= 2) {
            tagCountFactor = 0.5;
        }
        if (tagCount >= 3) {
            tagCountFactor = 0.3;
        }

        // Scale based on distance (closer = more confidence)
        double distanceFactor = Math.max(1.0, avgDistance / 2.0);

        // Scale based on robot speed (slower = more confidence)
        double speedFactor = 1.0 + robotSpeed;

        xyStdDev *= tagCountFactor * distanceFactor * speedFactor;
        thetaStdDev *= tagCountFactor * distanceFactor * speedFactor;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}
