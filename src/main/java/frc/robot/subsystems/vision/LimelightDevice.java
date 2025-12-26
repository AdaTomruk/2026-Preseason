// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems. vision;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math. Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi. first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry. Rotation2d;
import edu.wpi.first.math. numbers.N1;
import edu.wpi. first.math.numbers.N3;
import edu.wpi. first.wpilibj. DriverStation;
import edu. wpi.first.wpilibj.Timer;

import frc.robot.Constants. VisionConstants;
import frc.robot.Constants.VisionConstants.LimelightConstants;
import frc.robot. Constants.VisionConstants.LimelightModel;
import frc.robot. Constants.VisionConstants.LimelightRole;
import frc.robot. Constants.VisionConstants.PoseEstimationMethod;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot. util.VisionMeasurement;

public class LimelightDevice {
    private final String name;
    private final LimelightModel model;
    private final int id;
    private final LimelightRole role;
    private int[] tagFilter = new int[]{};

    // Standard deviation coefficient for translation
    private static final double kTranslationStdDevCoefficient = 0.01;

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

    /**
     * Set tag filter based on alliance color. 
     * This improves pose estimation by only looking at relevant tags.
     * 
     * @param alliance Current alliance (Red or Blue)
     */
    public void setTagFilter(Optional<DriverStation.Alliance> alliance) {
        if (alliance. isEmpty()) {
            tagFilter = new int[]{}; // Empty = all tags
        } else {
            switch (alliance.get()) {
                case Red:
                    // Red alliance tags (adjust these based on your field layout)
                    tagFilter = new int[]{3, 4, 5, 6, 7, 8};
                    break;
                case Blue:
                    // Blue alliance tags (adjust these based on your field layout)
                    tagFilter = new int[]{9, 10, 11, 12, 13, 14};
                    break;
            }
        }
        
        LimelightHelpers. SetFiducialIDFiltersOverride(name, tagFilter);
    }

    public double getTx() {
        return LimelightHelpers.getTX(name);
    }

    public double getTy() {
        return LimelightHelpers. getTY(name);
    }

    public boolean getTv() {
        return LimelightHelpers.getTV(name);
    }

    public ArrayList<Integer> getVisibleTags() {
        if (role == LimelightRole. NOTHING) return new ArrayList<Integer>();
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        ArrayList<Integer> visibleTags = new ArrayList<>();
        for (RawFiducial raw : fiducials) {
            visibleTags.add(raw. id);
        }
        return visibleTags;
    }

    public int[] getVisibleTagIDs() {
        if (role == LimelightRole. NOTHING) return new int[0];
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        return java.util.Arrays.stream(fiducials)
            .mapToInt(raw -> raw.id)
            .toArray();
    }

    /**
     * Get vision measurement from this Limelight device.
     * Intelligently selects between MegaTag1 and MegaTag2 based on conditions.
     * 
     * IMPORTANT: Call this with the robot's current heading for best results!
     * 
     * @param robotSpeed Current robot speed in meters per second
     * @param robotHeading Current robot heading (used to improve LL accuracy)
     * @return Optional VisionMeasurement if valid pose is available
     */
    public Optional<VisionMeasurement> getVisionMeasurement(double robotSpeed, Rotation2d robotHeading) {
        if (role == LimelightRole.NOTHING) return Optional.empty();

        // Update Limelight with robot orientation for better pose estimation
        // This is CRITICAL for accurate measurements!
        LimelightHelpers.SetRobotOrientation(
            name, 
            robotHeading.getDegrees(), 
            0, 0, 0, 0, 0
        );

        int[] tagIds = getVisibleTagIDs();
        if (tagIds.length == 0) {
            return Optional.empty();
        }

        // Determine which pose estimation method to use
        PoseEstimationMethod method = selectPoseEstimationMethod(tagIds. length, robotSpeed);
        PoseEstimate estimate;
        
        if (method == PoseEstimationMethod.MEGATAG_1) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        } else {
            estimate = LimelightHelpers. getBotPoseEstimate_wpiBlue_MegaTag2(name);
        }

        // Validate estimate
        if (! LimelightHelpers.validPoseEstimate(estimate)) {
            return Optional.empty();
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

        // Calculate standard deviations based on method, tag count, distance, and robot speed
        var stdDevs = calculateStdDevs(method, estimate.tagCount, avgDistance, robotSpeed);

        return Optional.of(new VisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            stdDevs,
            name,
            estimate.tagCount,
            avgDistance,
            robotSpeed,
            method. name()
        ));
    }

    /**
     * Overload for backward compatibility - uses default heading of 0
     */
    public Optional<VisionMeasurement> getVisionMeasurement(double robotSpeed) {
        return getVisionMeasurement(robotSpeed, new Rotation2d());
    }

    /**
     * Select the appropriate pose estimation method based on conditions.
     * 
     * MegaTag1 is used when:
     * - 2+ tags are visible
     * - Robot is moving slowly (< 0.5 m/s)
     * - Tags are close enough (< 3. 75 m)
     * 
     * Otherwise, MegaTag2 is used. 
     * 
     * MegaTag1 provides better rotation estimates but requires better conditions.
     * MegaTag2 is more robust but doesn't provide reliable rotation. 
     */
    private PoseEstimationMethod selectPoseEstimationMethod(int tagCount, double robotSpeed) {
        // Get MegaTag2 estimate to check distance
        PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        if (mt2Estimate == null) {
            return PoseEstimationMethod. MEGATAG_2;
        }

        double avgDistance = mt2Estimate. avgTagDist;
        
        // Criteria for using MegaTag1 (better rotation, but needs good conditions)
        boolean twoOrMoreTags = tagCount >= 2;
        boolean movingSlowly = robotSpeed <= VisionConstants.kMaxSpeedForMegaTag1;
        boolean closeEnough = avgDistance <= VisionConstants.kMaxDistanceForMegaTag1;
        
        boolean useMegaTag1 = twoOrMoreTags && movingSlowly && closeEnough;

        return useMegaTag1 ? PoseEstimationMethod.MEGATAG_1 : PoseEstimationMethod.MEGATAG_2;
    }

    /**
     * Calculate standard deviations for vision measurement.
     * Lower values = more trust in the measurement.
     * 
     * CRITICAL: MegaTag1 trusts rotation, MegaTag2 does NOT!
     * 
     * Standard deviations scale with:
     * - Fewer tags visible = higher std dev
     * - Greater distance = higher std dev (quadratic!)
     * - Higher robot speed = higher std dev
     * 
     * @param method Which pose estimation method was used
     * @param tagCount Number of tags visible
     * @param avgDistance Average distance to visible tags in meters
     * @param robotSpeed Current robot speed in meters per second
     * @return Matrix of standard deviations [x, y, theta]
     */
    private Matrix<N3, N1> calculateStdDevs(PoseEstimationMethod method, int tagCount, 
                                             double avgDistance, double robotSpeed) {
        
        // Calculate translation standard deviation using quadratic distance scaling
        // Formula: coefficient * distance^2 / tagCount
        double xyStdDev = kTranslationStdDevCoefficient * Math.pow(avgDistance, 2) / tagCount;
        
        // Apply speed scaling (faster = less trust)
        double speedFactor = 1.0 + robotSpeed;
        xyStdDev *= speedFactor;
        
        // Ensure minimum threshold
        xyStdDev = Math.max(xyStdDev, 0.05);

        // CRITICAL DIFFERENCE:  Rotation handling depends on method!
        double thetaStdDev;
        
        if (method == PoseEstimationMethod. MEGATAG_1) {
            // MegaTag1 with 2+ tags provides good rotation estimate
            thetaStdDev = 0.3;  // Trust this rotation! 
        } else {
            // MegaTag2 does NOT provide reliable rotation
            thetaStdDev = 9999999;  // Effectively ignore this rotation
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /**
     * Alternative calculation method matching your original style but with fixes.
     * You can use this instead if you prefer the factor-based approach.
     */
    @SuppressWarnings("unused")
    private Matrix<N3, N1> calculateStdDevsAlternative(PoseEstimationMethod method, int tagCount, 
                                                        double avgDistance, double robotSpeed) {
        
        // Base standard deviations
        double xyStdDev = 0.5;

        // Scale based on tag count (more tags = more confidence)
        double tagCountFactor = 1.0;
        if (tagCount >= 2) {
            tagCountFactor = 0.5;
        }
        if (tagCount >= 3) {
            tagCountFactor = 0.3;
        }

        // Scale based on distance - using quadratic for better accuracy
        double distanceFactor = Math.pow(avgDistance / 2.0, 2);

        // Scale based on robot speed (slower = more confidence)
        double speedFactor = 1.0 + robotSpeed;

        // Apply all factors to translation
        xyStdDev *= tagCountFactor * distanceFactor * speedFactor;
        
        // Ensure minimum threshold
        xyStdDev = Math.max(xyStdDev, 0.05);

        // CRITICAL:  Different theta handling per method
        double thetaStdDev;
        if (method == PoseEstimationMethod.MEGATAG_1) {
            thetaStdDev = 0.3;  // Trust rotation with 2+ tags
        } else {
            thetaStdDev = 9999999;  // Don't trust rotation from MegaTag2
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}