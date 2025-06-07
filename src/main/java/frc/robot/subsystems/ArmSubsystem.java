// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  // private Supplier<Angle> sim;
  private SparkMax motor;
  private ArmSim sim;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    motor = new SparkMax(10, MotorType.kBrushless);
    sim = new ArmSim(motor);
  }

  public void simulateVoltage(double volts) {
    if (sim != null) {
      sim.runSim(Volts.of(volts)); // ✅ This is the method you're missing
    }
  }

  // void setSimMethod(Supplier<Angle> simMethod) {
  //   sim = simMethod;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // Angle simAngle = sim.get();
    if (sim != null) {
      Angle simAngle = sim.runSim(Volts.of(12.0));
      Logger.recordOutput("Sim Arm Angle", simAngle.in(Degrees));
    }
  }
}
