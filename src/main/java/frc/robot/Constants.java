package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.Structures.LimelightConstants;

public class Constants {
    public static final class DriveConstants {
        public static final Distance autoApproachOffset = Meters.of(0.075);

        public static final class AutoConstants {
            public static final PIDConstants kTranslationPID = new PIDConstants(5.0,0,0);
            public static final PIDConstants kRotationPID = new PIDConstants(5.0,0,0);

            public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
                AutoConstants.kTranslationPID, 
                AutoConstants.kRotationPID
            );

            public RobotConfig PathPlanerConfig = new RobotConfig(
                Kilogram.of(35),
                KilogramSquareMeters.of(2.0), // CHANGED: Approximate MOI for a 28"x28" robot of this mass
                new ModuleConfig(
                    Inches.of(2.0), // Standard 4" Wheel Radius (MK5n uses 4" wheels)
                    MetersPerSecond.of(5.5), // CHANGED: Theoretical max speed (Depends on your specific gear ratio, see below)
                    1.1, // Wheel Coefficient of Friction (1.1-1.2 is standard for tread on carpet)
                    DCMotor.getKrakenX60(1), // CHANGED: Drive Motor is Kraken X60
                    6.75, // VERIFY THIS: The Drive Gear Ratio (See note below)
                    Amps.of(60), // CHANGED: Current limit (Kraken can take 60-80A safely)
                    1 // Number of motors per module
                ),
                // Module Offsets (Keep these matched to your CAD/Physical robot)
                new Translation2d(Inches.of(12.625), Inches.of(12.5625)),
                new Translation2d(Inches.of(12.125), Inches.of(-12.5)),
                new Translation2d(Inches.of(-12), Inches.of(12.5)),
                new Translation2d(Inches.of(-12.125), Inches.of(-12.4375))
            );

            public static final PPHolonomicDriveController kAutoAlignPIDController = new PPHolonomicDriveController(
                new PIDConstants(5.5, 0.0, 0.1, 0.0), 
                AutoConstants.kRotationPID
            );

            public static final Time kEndTriggerDebounce = Seconds.of(0.04);

            public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(3.0);
            public static final Distance kPositionTolerance = Centimeter.of(1.5);
            public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(2);

            public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
            public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);

            public static final PathConstraints kStartingPathConstraints = new PathConstraints(2.25, 2.20, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.
            public static final PathConstraints kTeleopPathConstraints = new PathConstraints(2.5, 2.0, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.
            public static final PathConstraints kAutoPathConstraints = new PathConstraints(2.25, 2.25, 1/2 * Math.PI, 1 * Math.PI); //? consider making these more aggressive

            }
        }

    public static final class VisionConstants {
        public static final class AprilTagRegions {
            public static final int[] RED_ALLIANCE = { 1, 2, 3, 4, 5, 6 };
            public static final int[] BLUE_ALLIANCE = { 1, 2, 3, 4, 5, 6 };

        }

        public static final LimelightConstants kLimelights[] = {
            new LimelightConstants("FrontLL", LimelightModel.LIMELIGHT_3G, 11, LimelightRole.LOCALIZATION)
        };

        public static final double kMaxAngularSpeed = 720;
        public static final double kMaxSpeedForMegaTag1 = 0.5; // meters
        public static final double kMaxDistanceForMegaTag1 = 3.75; // meters
        public static final double kMaxDistance = 8; // meters
        public static final boolean kVisionDiagnostics = true;

        public static final Time newMegaTag1ReadingThreshold = Seconds.of(10);

        public static final class OdometryConstants {
            public static final double kMaxSwerveVisionPoseDifference = 1.0; // meters
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
