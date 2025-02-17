// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeScorerConstants;
import utilities.DebugEntry;

public class CANAlgaeManipulatorSubsystem extends SubsystemBase {
  private final VictorSPX pivotMotor;
  private final VictorSPX rollerMotor;

  private final Encoder pivotEncoder;

  private final PIDController pivotPID;

  private DebugEntry<Double> pivotSetAngleEntry = new DebugEntry<Double>(0.0, "SetPivotAngle", this);
  private DebugEntry<Double> pivotMotorOutputEntry =
      new DebugEntry<Double>(0.0, "PivotMOtorOutput", this);
  private DebugEntry<Double> pivotAngleEntry =
      new DebugEntry<Double>(0.0, "PivotEncoderDistance", this);

  public CANAlgaeManipulatorSubsystem() {
    pivotMotor = new VictorSPX(AlgaeScorerConstants.PIVOT_MOTOR_ID);
    rollerMotor = new VictorSPX(AlgaeScorerConstants.ROLLER_MOTOR_ID);

    pivotMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.setNeutralMode(NeutralMode.Brake);

    pivotEncoder =
        new Encoder(
            AlgaeScorerConstants.PIVOT_ENCODER_A, AlgaeScorerConstants.PIVOT_ENCODER_B, false);

    pivotEncoder.setDistancePerPulse(360.0 / AlgaeScorerConstants.ENCODER_RESOLUTION);

    pivotPID =
        new PIDController(
            AlgaeScorerConstants.PivotPID.p,
            AlgaeScorerConstants.PivotPID.i,
            AlgaeScorerConstants.PivotPID.d);
    
  pivotPID.setSetpoint(0);

    
  }

  public void calculatePivotPID() {
    double targetAngle = pivotPID.getSetpoint();
    double currentAngle = pivotEncoder.getDistance();
    double PIDOutput = pivotPID.calculate(currentAngle, targetAngle);

    pivotMotor.set(VictorSPXControlMode.PercentOutput, PIDOutput);
  }

  public void SetRoller(double rollerMotorPercent) {
    rollerMotor.set(VictorSPXControlMode.PercentOutput, rollerMotorPercent);
  }

  public void setPivotAngle(Angle setAngle) {
    pivotPID.setSetpoint(setAngle.in(Degrees));
  }

  @Override
  public void periodic() {
    calculatePivotPID();
    pivotSetAngleEntry.log(pivotPID.getSetpoint());
    pivotMotorOutputEntry.log(pivotMotor.getMotorOutputPercent());
    pivotAngleEntry.log(pivotEncoder.getDistance());
  }

  public Command intakeAlgaeCommand() {
    return run(() -> setPivotAngle(AlgaeScorerConstants.PIVOT_INTAKE_ANGLE))
        .andThen(() -> SetRoller(AlgaeScorerConstants.INTAKE_SPEED_PERCENT), this);
  }

  public Command setIntakeAngleCommand(Angle intakeAngle) {
    return run(() -> setPivotAngle(intakeAngle));
  }

  public Command OutakeAlgaeCommand() {
    return run(() -> SetRoller(AlgaeScorerConstants.OUTAKE_TAKE_SPEED_PERCENT));
  }
}
