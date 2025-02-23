// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeScorerConstants;
import frc.robot.Robot;
import utilities.DebugEntry;
import utilities.HowdyPID;

public class CANAlgaeManipulatorSubsystem extends SubsystemBase {
  private final WPI_VictorSPX pivotMotor;
  private final SparkMax rollerMotor;

  private final VictorSPXSimCollection pivotMotorSim;

  private final SparkMaxConfig rollerConfig;

  private final Encoder pivotEncoder;
  private final EncoderSim pivotEncoderSim;

  private final HowdyPID pivotPID;

  private final SingleJointedArmSim pivotArmSim;

  private final Mechanism2d pivotMech = new Mechanism2d(2, 2);

  private final MechanismLigament2d pivotArmMechLigament;
  private final MechanismLigament2d pivotSetAngleMechLigament;

  private final ComplexWidget simWidget =
      Shuffleboard.getTab(getName()).add("Pivot Arm", pivotMech);

  private DebugEntry<Double> pivotSetAngleEntry =
      new DebugEntry<Double>(0.0, "SetPivotAngle", this);
  private DebugEntry<Double> pivotMotorOutputEntry =
      new DebugEntry<Double>(0.0, "PivotMotorOutput", this);
  private DebugEntry<Double> pivotAngleEntry =
      new DebugEntry<Double>(0.0, "PivotEncoderDistance", this);

  public CANAlgaeManipulatorSubsystem() {
    pivotMotor = new WPI_VictorSPX(AlgaeScorerConstants.PIVOT_MOTOR_ID);
    rollerMotor = new SparkMax(AlgaeScorerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    pivotMotor.setInverted(true);

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

    pivotEncoderSim = new EncoderSim(pivotEncoder);

    pivotPID =
        new HowdyPID(
            AlgaeScorerConstants.PivotPID.p,
            AlgaeScorerConstants.PivotPID.i,
            AlgaeScorerConstants.PivotPID.d);

    pivotPID.getPIDController().setSetpoint(AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Degrees));

    pivotPID.createTunableNumbers("PivotPID", pivotPID.getPIDController(), this);

    pivotMotorSim = new VictorSPXSimCollection(pivotMotor);

    pivotSetAngleMechLigament =
        pivotMech
            .getRoot("PivotSetAngle", 1, 1)
            .append(
                new MechanismLigament2d(
                    "PivotSetAngle",
                    1,
                    AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Degrees),
                    10.0,
                    new Color8Bit(Color.kRed)));

    pivotArmMechLigament =
        pivotMech
            .getRoot("CurrentPivotAngle", 1, 1)
            .append(
                new MechanismLigament2d(
                    "PivotArm",
                    1,
                    AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Degrees),
                    10.0,
                    new Color8Bit(Color.kGreen)));

    pivotArmSim =
        new SingleJointedArmSim(
            DCMotor.getCIM(1),
            AlgaeScorerConstants.GEARING,
            AlgaeScorerConstants.ARM_MOI.in(KilogramMetersSquaredPerSecond),
            AlgaeScorerConstants.ARM_LENGTH.in(Meters),
            AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Radians),
            AlgaeScorerConstants.PIVOT_INTAKE_ANGLE.in(Radians),
            true,
            AlgaeScorerConstants.PIVOT_STOW_ANGLE.in(Radians));
  }

  public void calculatePivotPID() {
    double targetAngle = getPIDSetpoint();
    double deadband = AlgaeScorerConstants.PIVOT_ANGLE_DEADBAND.in(Degrees);
    double currentAngle = getPivotAngle(Degrees);

    if (Math.abs(currentAngle - targetAngle) > deadband) {
      double PIDOutput = pivotPID.getPIDController().calculate(currentAngle, targetAngle);
      double limitedOutput = Math.max(-0.5, Math.min(0.5, PIDOutput));

      pivotMotor.set(VictorSPXControlMode.PercentOutput, limitedOutput);
    } else {
      pivotMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
  }

  public void setRoller(double rollerMotorPercent) {
    rollerMotor.set(rollerMotorPercent);
  }

  public void setPivotAngle(Angle setAngle) {
    pivotPID.getPIDController().setSetpoint(setAngle.in(Degrees));
  }

  public double getPivotAngle(AngleUnit unit) {
    if (Robot.isReal()) {
      return Degrees.of(
              pivotEncoder.getDistance()
                  + AlgaeScorerConstants.PIVOT_ENCODER_OFFSET_ANGLE.in(Degrees))
          .in(unit);
    } else if (Robot.isSimulation()) {
      return Degrees.of(
              pivotEncoderSim.getDistance()
                  + AlgaeScorerConstants.PIVOT_ENCODER_OFFSET_ANGLE.in(Degrees))
          .in(unit);
    } else {
      return 0;
    }
  }

  public double getPIDSetpoint() {
    return pivotPID.getPIDController().getSetpoint()
        + AlgaeScorerConstants.PIVOT_ENCODER_OFFSET_ANGLE.in(Degrees);
  }

  @Override
  public void periodic() {
    pivotSetAngleEntry.log(getPIDSetpoint());
    pivotMotorOutputEntry.log(pivotMotor.getMotorOutputPercent());
    pivotAngleEntry.log(getPivotAngle(Degrees));
    calculatePivotPID();

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void simulationPeriodic() {
    pivotArmSim.setInputVoltage(pivotMotor.getMotorOutputVoltage());
    pivotArmSim.update(0.02);
    pivotEncoderSim.setDistance(Radians.of(pivotArmSim.getAngleRads()).in(Degrees));

    pivotArmMechLigament.setAngle(getPivotAngle(Degrees));
    pivotSetAngleMechLigament.setAngle(getPIDSetpoint());
  }

  public Command intakeAlgaeCommand() {
    return run(() -> setPivotAngle(AlgaeScorerConstants.PIVOT_INTAKE_ANGLE))
        .andThen(() -> setRoller(AlgaeScorerConstants.INTAKE_SPEED_PERCENT), this)
        .withName("intakeAlgaeCommand");
  }

  public Command setIntakeAngleCommand(Angle intakeAngle) {
    return run(() -> setPivotAngle(intakeAngle)).withName("setIntakeAngleCommand");
  }

  public Command OutakeAlgaeCommand() {
    return run(() -> setRoller(AlgaeScorerConstants.OUTAKE_TAKE_SPEED_PERCENT))
        .withName("OutakeAlgaeCommand");
  }
}
