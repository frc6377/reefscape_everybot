// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANDriveSubsystem;

public final class Autos {
  public static Command exampleAuto(CANDriveSubsystem driveSubsystem) {
    return Commands.sequence(
        driveSubsystem.driveCommand(0.5),
        driveSubsystem.turnCommand(90),
        driveSubsystem.driveCommand(0.5),
        driveSubsystem.turnCommand(90));
  }

  public static Command rotateAuto(CANDriveSubsystem driveSubsystem) {
    return driveSubsystem.turnCommand(180);
  }
}
