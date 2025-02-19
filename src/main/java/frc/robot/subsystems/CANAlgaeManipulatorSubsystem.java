// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
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
import utilities.DebugEntry;
import utilities.HowdyPID;

public class CANAlgaeManipulatorSubsystem extends SubsystemBase {
  private final VictorSPX pivotMotor;
  private final VictorSPX rollerMotor;

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
    rollerMotor = new VictorSPX(AlgaeScorerConstants.ROLLER_MOTOR_ID);

    pivotMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.setNeutralMode(NeutralMode.Brake);

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

  public void calculatePivotPID() {
    double targetAngle = pivotPID.getPIDController().getSetpoint();
    double currentAngle = pivotEncoder.getDistance();
    double PIDOutput = pivotPID.getPIDController().calculate(currentAngle, targetAngle);

    // Limit PID output to prevent excessive movement
    double limitedOutput = Math.max(-0.5, Math.min(0.5, PIDOutput));
    pivotMotor.set(VictorSPXControlMode.PercentOutput, limitedOutput);
  }

  private Torque calculateGravityTorque(double armAngleRadians) {
    return NewtonMeters.of(
        AlgaeScorerConstants.ARM_MASS.in(Kilograms)
            * AlgaeScorerConstants.GRAVITY.in(MetersPerSecondPerSecond)
            * AlgaeScorerConstants.ARM_LENGTH.in(Meters)
            * Math.sin(armAngleRadians));
  }

  private Torque calculateMotorTorque() {
    double motorPercent = pivotMotor.getMotorOutputPercent();
    return NewtonMeters.of(motorPercent * AlgaeScorerConstants.MAX_ARM_TORQUE.in(NewtonMeters));
  }

  // NOTE: this method is very experimental, 90% chance won't work, pls remove from simPeriodic
  // before running
  private void applyGravity() {
    Time updateTime = Seconds.of(0.02);

    double gravityTorque = calculateGravityTorque(pivotEncoder.getDistance()).in(NewtonMeters);
    double motorTorque = calculateMotorTorque().in(NewtonMeters);

    Torque netTorque = NewtonMeters.of(motorTorque - gravityTorque);

    AngularVelocity currentVelocity = RadiansPerSecond.of(pivotEncoder.getRate());
    Angle currentAngle = Degrees.of(pivotEncoder.getDistance());

    AngularAcceleration angularAcceleration =
        RadiansPerSecondPerSecond.of(
            netTorque.in(NewtonMeters)
                / AlgaeScorerConstants.ARM_MOI.in(KilogramMetersSquaredPerSecond));

    AngularVelocity newVelocity = currentVelocity.plus(angularAcceleration.times(updateTime));

    Angle newAngle = currentAngle.plus(newVelocity.times(updateTime));

    pivotArmMech.setAngle(newAngle.in(Degrees));
  }

  public void setRoller(double rollerMotorPercent) {
    rollerMotor.set(VictorSPXControlMode.PercentOutput, rollerMotorPercent);
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
    calculatePivotPID();
    pivotArmMech.setAngle(
        pivotEncoder.getDistance() + AlgaeScorerConstants.PIVOT_SIM_OFFSET_ANGLE.in(Degrees));
    pivotSetAngleMech.setAngle(
        pivotPID.getPIDController().getSetpoint()
            + AlgaeScorerConstants.PIVOT_SIM_OFFSET_ANGLE.in(Degrees));
    // applyGravity();

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
