package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants.PoseEstimationMethod;

public class Structures {

    public static record VisionMeasurement(
        Pose2d pose,
        double timestamp,
        Matrix<N3, N1> stdDevs,
        String diagName,
        int diagTagCount,
        double diagTagDistance,
        double diagRobotSpeed,
        PoseEstimationMethod diagMethod
    ) {}

}
