// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    
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
