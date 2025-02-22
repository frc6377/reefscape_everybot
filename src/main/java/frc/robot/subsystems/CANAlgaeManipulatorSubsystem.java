// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeScorerConstants;
import frc.robot.Constants.CoralScorerConstants;
import utilities.DebugEntry;
import utilities.HowdyPID;

public class CANAlgaeManipulatorSubsystem extends SubsystemBase {
  private final VictorSPX pivotMotor;
  private final SparkMax rollerMotor;

  private final SparkMaxConfig rollerConfig;

  private final Encoder pivotEncoder;

  private final HowdyPID pivotPID;

  private final Mechanism2d pivotMech = new Mechanism2d(2, 2);

  private final MechanismLigament2d pivotArmMech;
  private final MechanismLigament2d pivotSetAngleMech;

  private final ComplexWidget simWidget =
      Shuffleboard.getTab(getName()).add("Pivot Arm", pivotMech);

  private DebugEntry<Double> pivotSetAngleEntry =
      new DebugEntry<Double>(0.0, "SetPivotAngle", this);
  private DebugEntry<Double> pivotMotorOutputEntry =
      new DebugEntry<Double>(0.0, "PivotMotorOutput", this);
  private DebugEntry<Double> pivotAngleEntry =
      new DebugEntry<Double>(0.0, "PivotEncoderDistance", this);

  public CANAlgaeManipulatorSubsystem() {
    pivotMotor = new VictorSPX(AlgaeScorerConstants.PIVOT_MOTOR_ID);
    rollerMotor = new SparkMax(AlgaeScorerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    pivotMotor.setNeutralMode(NeutralMode.Brake);
    
    rollerMotor.setCANTimeout(75);

    rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kBrake);
    rollerConfig.voltageCompensation(AlgaeScorerConstants.ROLLER_MOTOR_VOLTAGE_COMP.in(Volts));
    rollerConfig.smartCurrentLimit((int) AlgaeScorerConstants.ROLLER_MOTOR_CURRENT_LIMIT.in(Amps));
    
    
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder =
        new Encoder(
            AlgaeScorerConstants.PIVOT_ENCODER_A, AlgaeScorerConstants.PIVOT_ENCODER_B, true);

    pivotEncoder.setDistancePerPulse(360.0 / AlgaeScorerConstants.ENCODER_RESOLUTION);

    pivotPID =
        new HowdyPID(
            AlgaeScorerConstants.PivotPID.p,
            AlgaeScorerConstants.PivotPID.i,
            AlgaeScorerConstants.PivotPID.d);

    pivotPID.getPIDController().setSetpoint(AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Degrees));

    pivotPID.createTunableNumbers("PivotPID", pivotPID.getPIDController(), this);

    pivotSetAngleMech =
        pivotMech
            .getRoot("PivotSetAngle", 1, 1)
            .append(
                new MechanismLigament2d(
                    "PivotSetAngle",
                    1,
                    AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Degrees)
                        + AlgaeScorerConstants.PIVOT_SIM_OFFSET_ANGLE.in(Degrees),
                    10.0,
                    new Color8Bit(Color.kRed)));

    pivotArmMech =
        pivotMech
            .getRoot("CurrentPivotAngle", 1, 0)
            .append(
                new MechanismLigament2d(
                    "PivotArm",
                    1,
                    AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Degrees)
                        + AlgaeScorerConstants.PIVOT_SIM_OFFSET_ANGLE.in(Degrees),
                    10.0,
                    new Color8Bit(Color.kGreen)));
  }


  public void setRoller(double rollerMotorPercent) {
    rollerMotor.set(rollerMotorPercent);
  }

  public void setPivotAngle(Angle setAngle) {
    pivotPID.getPIDController().setSetpoint(setAngle.in(Degrees));
  }

  @Override
  public void periodic() {
    pivotSetAngleEntry.log(pivotPID.getPIDController().getSetpoint());
    pivotMotorOutputEntry.log(pivotMotor.getMotorOutputPercent());
    pivotAngleEntry.log(pivotEncoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {

  }

  public Command intakeAlgaeCommand() {
    return run(() -> setPivotAngle(AlgaeScorerConstants.PIVOT_INTAKE_ANGLE))
        .andThen(() -> setRoller(AlgaeScorerConstants.INTAKE_SPEED_PERCENT), this);
  }

  public Command setIntakeAngleCommand(Angle intakeAngle) {
    return run(() -> setPivotAngle(intakeAngle));
  }

  public Command OutakeAlgaeCommand() {
    return run(() -> setRoller(AlgaeScorerConstants.OUTAKE_TAKE_SPEED_PERCENT));
  }
}
