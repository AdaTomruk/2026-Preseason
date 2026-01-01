package frc.robot.subsystems.vision;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVisionSubsystem extends SubsystemBase {
    
    private static boolean mt1Override = false;

    public static boolean getMegaTag1Override() {
        return mt1Override;
    }

    public static void setMegaTag1Override(boolean b) {
        mt1Override = b;
    }
}
