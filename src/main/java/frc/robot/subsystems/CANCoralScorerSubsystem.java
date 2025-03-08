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
import frc.robot.Constants.CoralScorerConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Class to run the rollers over CAN */
public class CANCoralScorerSubsystem extends SubsystemBase {
  private final VictorSPX rollerMotor;
  private double intakeSpeed;

  public CANCoralScorerSubsystem() {
    rollerMotor = new VictorSPX(CoralScorerConstants.ROLLER_MOTOR_ID);
    rollerMotor.setInverted(true);
    intakeSpeed = CoralScorerConstants.ROLLER_EJECT_PERCENT_LOW;
  }

  public void toggleIntakeSpeed() {
    if (this.intakeSpeed == CoralScorerConstants.ROLLER_EJECT_PERCENT_HIGH) {
      this.intakeSpeed = CoralScorerConstants.ROLLER_EJECT_PERCENT_LOW;
    } else {
      this.intakeSpeed = CoralScorerConstants.ROLLER_EJECT_PERCENT_HIGH;
    }
  }

  public double getIntakeSpeed() {
    return intakeSpeed;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Coral Motor Ouput Percent", rollerMotor.getMotorOutputPercent());
    Logger.recordOutput("Coral Eject Speed", intakeSpeed);
  }

  // Run Roller at given speed
  public Command runRollerCommand(DoubleSupplier percent) {
    return startEnd(
        () -> rollerMotor.set(VictorSPXControlMode.PercentOutput, percent.getAsDouble()),
        () -> rollerMotor.set(VictorSPXControlMode.PercentOutput, 0));
  }

  // Scoring method
  public Command ejectCommand() {
    return runRollerCommand(() -> intakeSpeed);
  }

  public Command stopRoller() {
    return runRollerCommand(() -> 0.0);
  }

  public Command timedEjectCommand(Time ejectTime) {
    return Commands.deadline(Commands.waitSeconds(ejectTime.in(Seconds)), ejectCommand());
  }

  public Command toggleIntakeSpeedCommand() {
    return Commands.runOnce(() -> toggleIntakeSpeed(), this).withName("toggleIntakeSpeedCommand");
  }
}
