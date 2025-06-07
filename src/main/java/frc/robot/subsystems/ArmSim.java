// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmSimConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ArmSim {
  /** Creates a new ArmSim. */
  private SingleJointedArmSim armSim;

  private SparkMax motor;
  private SparkMaxSim motorSim;
  private Mechanism2d mech = new Mechanism2d(2, 2);
  private MechanismLigament2d armMech;

  public ArmSim(SparkMax spark) {
    motor = spark; // new SparkMax(10, MotorType.kBrushless);
    if (Robot.isSimulation()) {
      motorSim = new SparkMaxSim(motor, ArmSimConstants.kArmSimGearbox);
      armSim =
          new SingleJointedArmSim(
              ArmSimConstants.kArmSimGearbox,
              ArmSimConstants.kArmSimGearing,
              ArmSimConstants.kArmSimMOI,
              ArmSimConstants.kArmSimLength.in(Meters),
              ArmSimConstants.kStartAngle.in(Radians),
              ArmSimConstants.kEndAngle.in(Radians),
              true,
              0);
      motorSim.setPosition(
          (ArmSimConstants.kStartAngle).in(Rotations) * ArmSimConstants.kArmSimGearing);
      armMech =
          mech.getRoot("Arm Root", 1, 1)
              .append(
                  new MechanismLigament2d("Practice Arm", 1, 0, 10, new Color8Bit(Color.kBlue)));

      SmartDashboard.putData("Mech2Ds Arm Sim Mech", mech);
    }
    Logger.recordOutput("Practice Arm/Absolute Setpoint (Degrees)", Degrees.zero().in(Degrees));
  }

  // Stay in ArmSim
  public Angle runSim(Voltage input) {
    armSim.setInputVoltage(input.in(Volts));
    armSim.update(0.02);
    armMech.setAngle(armSim.getAngleRads() * 180 / Math.PI);
    return Radians.of(armSim.getAngleRads());
  }
}
