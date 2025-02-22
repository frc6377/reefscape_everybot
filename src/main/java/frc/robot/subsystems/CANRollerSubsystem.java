// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  private final VictorSPX rollerMotor;

  public CANRollerSubsystem() {
    rollerMotor = new VictorSPX(RollerConstants.ROLLER_MOTOR_ID);
    
  }

  @Override
  public void periodic() {}

  // Run Roller at given speed
  public Command runRollerCommand(double percent) {
    return startEnd(() -> rollerMotor.set(VictorSPXControlMode.PercentOutput, percent), () -> rollerMotor.set(VictorSPXControlMode.PercentOutput, 0));
  }

  // Scoring method
  public Command ejectCommand() {
    return runRollerCommand(RollerConstants.ROLLER_EJECT_PERCENT);
  }

  public Command intakeCommand() {
    return runRollerCommand(RollerConstants.ROLLER_INTAKE_PERCENT);
  }

  public Command stopRoller() {
    return runRollerCommand(0.0);
  }

  public Command timedEjectCommand(Time ejectTime) {
    return Commands.deadline(Commands.waitSeconds(ejectTime.in(Seconds)), ejectCommand());
  }
}
