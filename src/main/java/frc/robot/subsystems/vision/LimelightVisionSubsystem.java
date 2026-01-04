package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.logging.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.PoseEstimationMethod;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Structures.LimelightConstants;
import frc.robot.util.Structures.VisionMeasurement;

public class LimelightVisionSubsystem extends SubsystemBase {
    private final ArrayList<LimelightDevice> limelights;

    // Debugging settings
    private static boolean mt1Override = false;
    private static boolean discardVisionMeasurements = false;

    // Limelights (just using one for now)
    private LimelightDevice frontLL;

    private final CommandSwerveDrivetrain drivetrain;
    private final StructArrayPublisher<Pose3d> visionTargetPublisher;
    private final AprilTagFieldLayout fieldLayout;

    private boolean initalPoseSet = false;

    private long lastMegaTag1Reading;
    private boolean isMegaTag1ReadingNew = false;

    private long lastMegaTag2Reading;

    public LimelightVisionSubsystem(CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout fieldLayout) {
        limelights = new ArrayList<>();
        for (LimelightConstants llc : VisionConstants.kLimelights) {
            LimelightDevice llDevice = new LimelightDevice(llc);
            limelights.add(llDevice);
            boolean diagnosticsNeeded = VisionConstants.kVisionDiagnostics;
            switch (llc.role()) {
                case LOCALIZATION: {
                    frontLL = llDevice;
                    System.out.println("Setting front limelight to " + llc.id());
                    break;
                }
                case CAMERAFEED: {
                    System.out.println("Limelight " + llc.id() + " set as camera feed.");
                    // No setup yet
                    break;
                }
                default: {
                    diagnosticsNeeded = false;
                    System.out.println("Limelight " + llc.id() + " has no valid role!");
                    break;
                }
            }

            if (diagnosticsNeeded) {
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + llc.name() + "/stddev", -1);
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + llc.name() + "/count", -1);
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + llc.name() + "/distance", -1);
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + llc.name() + "/speed", -1);
                SmartDashboard.putString("VisionDiagnostics/limelight-" + llc.name() + "/method", "");
                SmartDashboard.putData("VisionDiagnostics/limelight-" + llc.name() + "/pose", new Field2d());
            }

            SmartDashboard.putBoolean("Initial Pose Set?", false);
            SmartDashboard.putBoolean("VisionDiagnostics/Want New MT1 Reading?", false);

            lastMegaTag1Reading = System.currentTimeMillis();
            lastMegaTag2Reading = System.currentTimeMillis();
        }

        this.fieldLayout = fieldLayout;
        this.drivetrain = drivetrain;

        if (VisionConstants.kVisionDiagnostics) {
            NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
            StructArrayTopic<Pose3d> visionTargetTopic = networkTableInstance.getStructArrayTopic(
                "VisionDiagnostics/vision targets", Pose3d.struct);
            visionTargetPublisher = visionTargetTopic.publish();
        } 
        else {
            visionTargetPublisher = null;
        }

        updateTagFilters();

    }

    private void updateTagFilters() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        limelights.forEach(ll -> ll.setTagFilter(alliance));
    }
    
    public ArrayList<VisionMeasurement> getVisionMeasurements() {
        ArrayList<VisionMeasurement> measurements = new ArrayList<>();
        limelights.forEach((limelight) -> {
            Optional<VisionMeasurement> measurement = limelight.getVisionMeasurement(drivetrain);
            if (measurement.isPresent()) {
                measurements.add(measurement.get());
            }
        });
        return measurements;
    }

    public boolean canSeeTags(String limelightName) {
        for (LimelightDevice ll : limelights) {
            if (ll.getName().equals(limelightName)) {
                return ll.getTv();
            }
        }
        return false;
    }

    public ArrayList<Integer> getVisibleTagIDs() {
        Set<Integer> set = new HashSet<>();
        limelights.forEach((limelight) -> {
            ArrayList<Integer> results = limelight.getVisibleTagIDs();
            set.addAll(results);
        });
        return new ArrayList<Integer>(set);
    } 

    public ArrayList<Pose3d> getVisibleTagPoses() {
        ArrayList<Integer> visibleTagIDs = getVisibleTagIDs();
        ArrayList<Pose3d> visibleTagPoses = new ArrayList<>();
        visibleTagIDs.forEach((Integer tagID) -> {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);
            if (tagPose.isEmpty()) {
                Logger.getLogger(this.getClass().getName())
                        .warning("Tag ID " + tagID + " not found in field layout ");
            } else {
                visibleTagPoses.add(tagPose.get());
            }
        });
        return visibleTagPoses;
    }

    @Override
    public void periodic() {
        Long currentTime = System.currentTimeMillis();
        Time sinceLastMegatag1Reading = Milliseconds.of(currentTime - lastMegaTag1Reading);  
        Time sinceLastMegatag2Reading = Milliseconds.of(currentTime - lastMegaTag2Reading);
        boolean wantNewMegaTag1Reading = sinceLastMegatag1Reading.gte(VisionConstants.newMegaTag1ReadingThreshold);
        SmartDashboard.putBoolean("VisionDiagnostics/Want New MT1 Reading?", wantNewMegaTag1Reading);

        SmartDashboard.putNumber("Since Last Megatag1 Reading", currentTime - lastMegaTag1Reading);
        SmartDashboard.putNumber("Since Last Megatag2 Reading", currentTime - lastMegaTag2Reading);

        getVisionMeasurements().forEach((measurement) -> {    
            if (!initalPoseSet){
                initalPoseSet = true;
                SmartDashboard.putBoolean("Initial Pose Set?", true);
            }
            if (measurement.diagMethod().equals(PoseEstimationMethod.MEGATAG_1)) {
                if (wantNewMegaTag1Reading) isMegaTag1ReadingNew = true;
                lastMegaTag1Reading = currentTime;
            }
            if (!discardVisionMeasurements){
                drivetrain.addVisionMeasurement(measurement.pose(), measurement.timestamp(), measurement.stdDevs());
            }
            if (VisionConstants.kVisionDiagnostics) {
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/stddev", measurement.stdDevs().get(0, 0));
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/count", measurement.diagTagCount());
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/distance", measurement.diagTagDistance());
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/speed", measurement.diagRobotSpeed());
                SmartDashboard.putString("VisionDiagnostics/" + measurement.diagName() + "/method", measurement.diagMethod().toString());
                ((Field2d) SmartDashboard.getData("VisionDiagnostics/" + measurement.diagName() + "/pose")).setRobotPose(measurement.pose());
            }
        });

        if (VisionConstants.kVisionDiagnostics) visionTargetPublisher.set(getVisibleTagPoses().toArray(new Pose3d[0]));
    }

    public static boolean getMegaTag1Override() {
        return mt1Override;
    }

    public static void setMegaTag1Override(boolean b) {
        mt1Override = b;
    }

    public static void setDiscardMeasurements(boolean b) {
        discardVisionMeasurements = b;
    }

    public boolean isInitialPoseSet() {
        return initalPoseSet;
    }

    public boolean newMegaTag1Reading() {
        if (isMegaTag1ReadingNew) {
            isMegaTag1ReadingNew = false;
            return true;
        }
        return false;
    }

}
