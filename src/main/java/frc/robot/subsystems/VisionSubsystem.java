// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;
import utilities.LimelightHelpers;

// README This is made only for apriltags not object detection yet

public class VisionSubsystem extends SubsystemBase {

  public VisionSubsystem() {}

  public void switchPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(VisionConstants.CAMERA_NAME, pipeline);
  }

  public void switchPipeline(VisionConstants.Pipelines pipeline) {
    switchPipeline(pipeline);
  }

  public void getCurrentPipeline() {
    LimelightHelpers.getCurrentPipelineIndex(VisionConstants.CAMERA_NAME);
  }

  public Pose2d getTargetRelativeRobotPose() {
    if (LimelightHelpers.getTV(VisionConstants.CAMERA_NAME)) {
      double[] poseList = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.CAMERA_NAME);
      return new Pose2d(poseList[0], poseList[1], Rotation2d.fromDegrees(poseList[4]));
    } else {
      return null;
    }
  }

  public Pose2d getRobotRelativeTargetPose() {
    if (LimelightHelpers.getTV(VisionConstants.CAMERA_NAME)) {
      double[] poseList = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.CAMERA_NAME);
      return new Pose2d(poseList[0], poseList[1], Rotation2d.fromDegrees(poseList[4]));
    } else {
      return null;
    }
  }

  public double getTargetID() {
    if (LimelightHelpers.getTV(VisionConstants.CAMERA_NAME)) {
      return LimelightHelpers.getFiducialID(VisionConstants.CAMERA_NAME);
    } else {
      return -1;
    }
  }

  public boolean isTargetVisible() {
    return LimelightHelpers.getTV(VisionConstants.CAMERA_NAME);
  }

  public Pose2d getGlobalPosefromReefAprilTag(double apriltagID) {
    if (VisionConstants.REEF_APRILTAG_LOCATIONS.keySet().contains(apriltagID)) {
      Pose2d apriltagPose = VisionConstants.REEF_APRILTAG_LOCATIONS.get(apriltagID);
      Pose2d robotPose = getTargetRelativeRobotPose();
      Pose2d globalPose =
          new Pose2d(
              apriltagPose.getX() + robotPose.getX(),
              apriltagPose.getY() + robotPose.getY(),
              apriltagPose.getRotation().plus(robotPose.getRotation()));
      return globalPose;
    } else {
      return null;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Vision/TargetRelativeRobotPose", getTargetRelativeRobotPose());
    Logger.recordOutput("Vision/RobotRelativeTargetPose", getRobotRelativeTargetPose());
    Logger.recordOutput("Vision/TargetVisible", isTargetVisible());
    Logger.recordOutput("Vision/TargetID", getTargetID());
  }
}
