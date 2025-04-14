// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import utilities.LimelightHelpers;

//README This is made only for apriltags not object detection yet

public class VisionSubsystem extends SubsystemBase {

  public static class Pipelines {
    public static final int APRILTAG = VisionConstants.APRILTAG_PIPELINE;
    public static final int OBJECT_DETECTION = VisionConstants.OBJECT_DETECTION_PIPELINE;
  }

  
  public VisionSubsystem() {}

  public static void switchPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(VisionConstants.CAMERA_NAME, pipeline);
  }

  public static void switchPipeline(Pipelines pipeline) {
    switchPipeline(pipeline);
  }

  public static void getCurrentPipeline() {
    LimelightHelpers.getCurrentPipelineIndex(VisionConstants.CAMERA_NAME);
  }

  public static Pose2d getTargetRelativeRobotPose() {
    double[] poseList  = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.CAMERA_NAME);
    return new Pose2d(poseList[0], poseList[1], Rotation2d.fromDegrees(poseList[4]));
  }

  public static Pose2d getRobotRelativeTargetPose(int pipeline) {
    double[] poseList  = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.CAMERA_NAME);
    return new Pose2d(poseList[0], poseList[1], Rotation2d.fromDegrees(poseList[4]));
  }


}
