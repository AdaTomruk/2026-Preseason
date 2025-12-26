// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * SimVisionSubsystem provides PhotonVision simulation support for testing
 * vision-based localization in simulation.
 */
public class SimVisionSubsystem extends SubsystemBase implements VisionDeviceSubsystem {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Optional<EstimatedRobotPose> latestPoseEstimate;
    private ArrayList<Integer> fiducialIds;

    public SimVisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        visionSim = new VisionSystemSim("main");

        // Load the 2025 field layout with AprilTags
        AprilTagFieldLayout tagLayout;
        try {
            tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        } catch (Exception e) {
            // If 2025 field not available, use a default field layout
            System.err.println("Could not load 2025 Reefscape field, using 2024 Crescendo as fallback: " + e.getMessage());
            try {
                tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
            } catch (Exception e2) {
                // Last resort: create an empty field layout
                System.err.println("Could not load any field layout: " + e2.getMessage());
                throw new RuntimeException("Failed to load AprilTag field layout for simulation", e2);
            }
        }
        visionSim.addAprilTags(tagLayout);

        // Create camera properties for simulation
        SimCameraProperties cameraProp = new SimCameraProperties();
        
        // The PhotonCamera used in the simulation
        camera = new PhotonCamera("SimCamera");

        // The simulation of this camera
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

        // Camera is mounted 0.5 meters up from the robot center and tilted down 15 degrees
        Translation3d robotToCameraTrl = new Translation3d(0.0, 0, 0.5);
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Create pose estimator with multi-tag strategy
        photonPoseEstimator = new PhotonPoseEstimator(
            tagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Add this camera to the vision system simulation
        visionSim.addCamera(cameraSim, robotToCamera);
        cameraSim.enableDrawWireframe(true);

        latestPoseEstimate = Optional.empty();
        fiducialIds = new ArrayList<>();
    }

    @Override
    public ArrayList<Integer> getVisibleTagIDs() {
        return fiducialIds;
    }

    @Override
    public Optional<Pose2d> getBotPose2dFromCamera() {
        if (latestPoseEstimate.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose estimatedPose = latestPoseEstimate.get();
        Pose2d result = estimatedPose.estimatedPose.toPose2d();

        return Optional.of(result);
    }
       
    @Override
    public void simulationPeriodic() {
        // Get the current robot pose from the drivetrain
        Pose2d simPose = drivetrain.getState().Pose;

        // Update the vision simulation with the robot's current pose
        visionSim.update(simPose);

        // Process all unread camera results
        var results = camera.getAllUnreadResults();
        fiducialIds = new ArrayList<>();

        if (!results.isEmpty()) {
            // Process each result and keep updating the estimate
            // The last result will be the most recent one
            for (var result : results) {
                Optional<EstimatedRobotPose> estimate = photonPoseEstimator.update(result);
                if (estimate.isPresent()) {
                    latestPoseEstimate = estimate;
                }
            }
            
            // Extract fiducial IDs from the first result
            PhotonPipelineResult firstResult = results.get(0);
            List<PhotonTrackedTarget> targetList = firstResult.getTargets();
            for (var target : targetList) {
                fiducialIds.add(target.getFiducialId());
            }
            
            // Add vision measurement to drivetrain if we have a valid estimate
            if (latestPoseEstimate.isPresent()) {
                EstimatedRobotPose estimate = latestPoseEstimate.get();
                drivetrain.addVisionMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds
                );
            }
        }
    }
}
