// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

public class CANDriveSubsystem extends SubsystemBase {
  private final VictorSPX leftLeader;
  private final VictorSPX leftFollower;
  private final VictorSPX rightLeader;
  private final VictorSPX rightFollower;

  private final DifferentialDrive diffDrive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new VictorSPX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    rightLeader.setInverted(false);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);

    // Create new Differntial Drive
    diffDrive =
        new DifferentialDrive(
            (speed) -> leftLeader.set(ControlMode.PercentOutput, speed),
            (speed) -> rightLeader.set(ControlMode.PercentOutput, speed));
  }

  @Override
  public void periodic() {}

  /**
   * @param forward how much to drive intake forward
   */
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(() -> diffDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()));
  }
}
