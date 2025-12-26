// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.VisionMeasurement;

public class LimelightVisionSubsystem extends SubsystemBase implements VisionDeviceSubsystem {
    private final ArrayList<LimelightDevice> limelights;
    private final CommandSwerveDrivetrain drivetrain;

    public LimelightVisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.limelights = new ArrayList<>();

        // Initialize all configured Limelights
        for (var config : VisionConstants.kLimelights) {
            limelights.add(new LimelightDevice(config));
        }
    }

    @Override
    public void periodic() {
        // Update robot orientation for all Limelights
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        
        for (var limelight : limelights) {
            LimelightHelpers.SetRobotOrientation(
                limelight.getName(),
                headingDeg,
                0, 0, 0, 0, 0
            );
        }

        // Get and process vision measurements
        var measurements = getVisionMeasurements();
        for (var measurement : measurements) {
            drivetrain.addVisionMeasurement(
                measurement.pose(),
                measurement.timestamp(),
                measurement.stdDevs()
            );

            // Publish diagnostics if enabled
            if (VisionConstants.kVisionDiagnostics) {
                publishDiagnostics(measurement);
            }
        }
    }

    /**
     * Get all valid vision measurements from all Limelights.
     * 
     * @return ArrayList of VisionMeasurement objects
     */
    public ArrayList<VisionMeasurement> getVisionMeasurements() {
        ArrayList<VisionMeasurement> measurements = new ArrayList<>();
        double robotSpeed = getRobotSpeed();

        for (var limelight : limelights) {
            var measurement = limelight.getVisionMeasurement(robotSpeed);
            measurement.ifPresent(measurements::add);
        }

        return measurements;
    }

    /**
     * Check if any Limelight can see tags.
     * 
     * @return true if any Limelight sees at least one tag
     */
    public boolean canSeeTags() {
        for (var limelight : limelights) {
            if (limelight.getVisibleTagIDs().length > 0) {
                return true;
            }
        }
        return false;
    }

    @Override
    public ArrayList<Integer> getVisibleTagIDs() {
        HashSet<Integer> uniqueTagIds = new HashSet<>();
        
        for (var limelight : limelights) {
            int[] tagIds = limelight.getVisibleTagIDs();
            for (int id : tagIds) {
                uniqueTagIds.add(id);
            }
        }
        
        return new ArrayList<>(uniqueTagIds);
    }

    @Override
    public Optional<Pose2d> getBotPose2dFromCamera() {
        // Return the first available pose from any camera
        double robotSpeed = getRobotSpeed();
        
        for (var limelight : limelights) {
            var measurement = limelight.getVisionMeasurement(robotSpeed);
            if (measurement.isPresent()) {
                return Optional.of(measurement.get().pose());
            }
        }
        
        return Optional.empty();
    }

    /**
     * Calculate the current robot speed from drivetrain state.
     * 
     * @return Robot speed in meters per second
     */
    private double getRobotSpeed() {
        var driveState = drivetrain.getState();
        return Math.hypot(
            driveState.Speeds.vxMetersPerSecond,
            driveState.Speeds.vyMetersPerSecond
        );
    }

    /**
     * Publish diagnostics to SmartDashboard for a vision measurement.
     */
    private void publishDiagnostics(VisionMeasurement measurement) {
        String prefix = "Vision/" + measurement.cameraName() + "/";
        
        SmartDashboard.putNumber(prefix + "TagCount", measurement.tagCount());
        SmartDashboard.putNumber(prefix + "AvgDistance", measurement.avgTagDistance());
        SmartDashboard.putNumber(prefix + "RobotSpeed", measurement.robotSpeed());
        SmartDashboard.putString(prefix + "Method", measurement.method());
        SmartDashboard.putNumber(prefix + "StdDevX", measurement.stdDevs().get(0, 0));
        SmartDashboard.putNumber(prefix + "StdDevY", measurement.stdDevs().get(1, 0));
        SmartDashboard.putNumber(prefix + "StdDevTheta", measurement.stdDevs().get(2, 0));
        SmartDashboard.putString(prefix + "Pose", measurement.pose().toString());
    }

    /**
     * Get all Limelight devices.
     * 
     * @return ArrayList of LimelightDevice objects
     */
    public ArrayList<LimelightDevice> getLimelights() {
        return limelights;
    }
}
