package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class Constants {
    public static final class DriveConstants {
        
    }

    public static final class VisionConstants {
        public static final class AprilTagRegions {
            public static final int[] RED_ALLIANCE = {1, 2, 3, 4, 5, 6};
            public static final int[] BLUE_ALLIANCE = {1, 2, 3, 4, 5, 6};
        
            
        }

        public static final double kMaxAngularSpeed = 720;
        public static final double kMaxSpeedForMegaTag1 = 0.5; //meters
        public static final double kMaxDistanceForMegaTag1 = 3.75; //meters
        public static final double kMaxDistance = 8;  //meters
        public static final boolean kVisionDiagnostics = true;
        
        public static final Time newMegaTag1ReadingThreshold = Seconds.of(10);

        public static final class OdometryConstants {
            public static final double kMaxSwerveVisionPoseDifference = 1.0; //meters
        }

        public enum LimelightModel {
            LIMELIGHT_3, LIMELIGHT_3G, LIMELIGHT_4
        }
        public enum LimelightRole {
            CAMERAFEED, LOCALIZATION
        }
        public enum PoseEstimationMethod {
            MEGATAG_1, MEGATAG_2
        }

        public static final class StdDevConstants {
            public static final class MegaTag1 {
                public static final double kInitialValue = 0.3;
                public static final double kTagCountReward = 0.15;
                public static final double kAverageDistancePunishment = 0.1;
                public static final double kRobotSpeedPunishment = 0.15;
                public static final double kSingleTagPunishment = 0.3;
            }
            public static final class MegaTag2 {
                public static final double kInitialValue = 0.2;
                public static final double kAverageDistancePunishment = 0.075;
                public static final double kRobotSpeedPunishment = 0.25;
                public static final double kMultipleTagsBonus = 0.05;
            }
        }


    }
    
}
