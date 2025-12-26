// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

public final class Constants {
    
    public static final class AutoConstants {
        // PathPlanner path constraints for auto and teleop
        public static final PathConstraints kAutoPathConstraints = new PathConstraints(
            MetersPerSecond.of(4.0),           // Max velocity
            MetersPerSecondPerSecond.of(3.0),  // Max acceleration
            MetersPerSecond.of(Math.toRadians(540)),      // Max angular velocity
            MetersPerSecondPerSecond.of(Math.toRadians(720)) // Max angular acceleration
        );
        
        public static final PathConstraints kTeleopPathConstraints = new PathConstraints(
            MetersPerSecond.of(3.0),           // Max velocity (slower for teleop)
            MetersPerSecondPerSecond.of(2.0),  // Max acceleration (slower for teleop)
            MetersPerSecond.of(Math.toRadians(360)),      // Max angular velocity
            MetersPerSecondPerSecond.of(Math.toRadians(540)) // Max angular acceleration
        );
        
        // Trapezoidal profile constraints for DriveToPointCommand
        public static final TrapezoidProfile.Constraints kDriveToPointConstraints = 
            new TrapezoidProfile.Constraints(4.0, 3.0); // Max vel: 4 m/s, Max accel: 3 m/s^2
        
        // Tolerances and timeouts
        public static final double kDriveToPointTolerance = 0.1; // meters
        public static final double kMinimumDriveSpeed = 0.3; // meters per second
        public static final double kAutoAlignAdjustTimeout = 1.5; // seconds
        public static final double kTeleopAlignAdjustTimeout = 2.0; // seconds
    }
    
    public static final class VisionConstants {
        public static final boolean kVisionDiagnostics = false;
        public static final double kMaxSpeedForMegaTag1 = 0.5; // meters per second
        public static final double kMaxDistanceForMegaTag1 = 3.75; // meters
        public static final double kMaxDistance = 8.0; // meters
        
        // Limelight configurations - adjust IDs and names as needed
        public static final record LimelightConstants(
            String name,
            LimelightModel model,
            int id,
            LimelightRole role
        ) {}
        
        public static final LimelightConstants[] kLimelights = {
            new LimelightConstants("limelight", LimelightModel.LIMELIGHT_3, 11, LimelightRole.MAIN)
        };
        
        public enum LimelightModel {
            LIMELIGHT_3, LIMELIGHT_3G
        }
        
        public enum LimelightRole {
            NOTHING, MAIN, ALIGN, STATION
        }
        
        public enum PoseEstimationMethod {
            MEGATAG_1, MEGATAG_2
        }
    }
}
