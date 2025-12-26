// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionDeviceSubsystem {
    ArrayList<Integer> getVisibleTagIDs();
    Optional<Pose2d> getBotPose2dFromCamera();
}
