// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANDriveSubsystem;

public final class Autos {

  public Command OneMeterAuto(CANDriveSubsystem drive) {
    return drive.driveXAxis(1);
  }

  public Command NinetyDegreeAuto(CANDriveSubsystem drive) {
    return drive.turnCommand(90);
  }

  public Command HalfMeterSquare(CANDriveSubsystem drive) {
    return Commands.sequence(
        drive.driveXAxis(1),
        drive.turnCommand(90),
        drive.driveXAxis(1),
        drive.turnCommand(90),
        drive.driveXAxis(1),
        drive.turnCommand(90),
        drive.driveXAxis(1),
        drive.turnCommand(90));
  }
}
