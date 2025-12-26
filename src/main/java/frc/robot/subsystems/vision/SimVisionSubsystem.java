// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SimVisionSubsystem extends SubsystemBase implements VisionDeviceSubsystem {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSystemSim visionSim;
    private final PhotonCamera reefCamera;
    private final PhotonCameraSim reefCameraSim;
    private final PhotonPoseEstimator photonEstimator;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    
    private ArrayList<Integer> visibleFiducialIDs;

    public SimVisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.visibleFiducialIDs = new ArrayList<>();

        // Load AprilTag field layout
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }

        // Create vision system simulation
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(aprilTagFieldLayout);

        // Create reef camera
        reefCamera = new PhotonCamera("ReefCamera");

        // Configure camera properties
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(960, 720, Rotation3d.kZero);
        cameraProperties.setCalibError(0.35, 0.10);
        cameraProperties.setFPS(15);
        cameraProperties.setAvgLatencyMs(50);
        cameraProperties.setLatencyStdDevMs(15);

        // Create camera simulation
        reefCameraSim = new PhotonCameraSim(reefCamera, cameraProperties);
        
        // Camera transform: 0.1m forward, 0.5m up, pitched 15° up from robot center
        Transform3d robotToCam = new Transform3d(
            new Translation3d(0.1, 0.0, 0.5),
            new Rotation3d(0, Math.toRadians(-15), 0)
        );
        
        visionSim.addCamera(reefCameraSim, robotToCam);
        reefCameraSim.enableDrawWireframe(true);

        // Create pose estimator with MULTI_TAG_PNP_ON_COPROCESSOR strategy
        photonEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam
        );
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        // Update simulation with current robot pose
        visionSim.update(drivetrain.getState().Pose);

        // Get latest camera result
        PhotonPipelineResult result = reefCamera.getLatestResult();
        
        // Update visible fiducial IDs
        visibleFiducialIDs.clear();
        if (result.hasTargets()) {
            result.getTargets().forEach(target -> 
                visibleFiducialIDs.add(target.getFiducialId())
            );
        }

        // Get pose estimate from PhotonVision
        var poseEstimate = photonEstimator.update(result);
        
        if (poseEstimate.isPresent()) {
            var estimate = poseEstimate.get();
            
            // Add vision measurement to drivetrain
            // Use simple std devs for simulation
            drivetrain.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(),
                estimate.timestampSeconds
            );
        }
    }

    @Override
    public ArrayList<Integer> getVisibleTagIDs() {
        return new ArrayList<>(visibleFiducialIDs);
    }

    @Override
    public Optional<Pose2d> getBotPose2dFromCamera() {
        PhotonPipelineResult result = reefCamera.getLatestResult();
        var poseEstimate = photonEstimator.update(result);
        
        if (poseEstimate.isPresent()) {
            return Optional.of(poseEstimate.get().estimatedPose.toPose2d());
        }
        
        return Optional.empty();
    }

    /**
     * Get the vision system simulation for debugging.
     */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    /**
     * Get the reef camera for debugging.
     */
    public PhotonCamera getReefCamera() {
        return reefCamera;
    }
}
