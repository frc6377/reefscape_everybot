// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 1;
    public static final int RIGHT_FOLLOWER_ID = 2;

    public static final int RIGHT_DRIVE_ENCODER_A = 0;
    public static final int RIGHT_DRIVE_ENCODER_B = 1;

    public static final int LEFT_DRIVE_ENCODER_A = 2;
    public static final int LEFT_DRIVE_ENCODER_B = 3;

    public static final int PIGEON_DEVICE_ID = 5;

    public static final Distance TRACK_WIDTH = Inches.of(22.5);

    public static final double GEARING = 8.45;

    public static final Distance WHEEL_DIAMETER = Inches.of(6);

    public static final LinearVelocity MAX_DRIVE_VELOCITY = MetersPerSecond.of(3.9);

    public static final LinearVelocity MIN_DRIVE_VELOCITY = MetersPerSecond.of(0);

    public static final AngularVelocity MAX_ROTATIONAL_VELOCITY = RadiansPerSecond.of(12.4);

    public static final AngularVelocity MIN_ROTATIONAL_VELOCITY = RadiansPerSecond.of(0);

    public static final double WHEEL_COF = 0.975;

    public static final Current MOTOR_CURRENT_LIMIT = Amps.of(40);

    public static final MomentOfInertia MOI = KilogramSquareMeters.of(7.80899009276);

    public static final Mass MASS = Pounds.of(120.194);

    public static final double ENCODER_RESOLUTION = 2048;

    public static final double CONTROL_CURVE_INTENSITY = 0.5;

    public static final class DrivePID {
      public static final double kp = 1;
      public static final double ki = 0.01;
      public static final double kd = 0.05;
    }

    public static final class RotatePID {
      public static final double kp = 0.01;
      public static final double ki = 0.01;
      public static final double kd = 0.01;
    }

    public static final double minPower = 0.3;
    public static final double debounce = 1;
  }

  public static final class CoralScorerConstants {
    public static final int ROLLER_MOTOR_ID = 6;
    public static final Current ROLLER_MOTOR_CURRENT_LIMIT = Amps.of(40);
    public static final Voltage ROLLER_MOTOR_VOLTAGE_COMP = Volts.of(12);
    public static final double ROLLER_EJECT_PERCENT_LOW = 0.44;
    public static final double ROLLER_EJECT_PERCENT_HIGH = 0.55;
    public static final double ROLLER_INTAKE_PERCENT = -0.54;

    public static final Time EJECT_TIME = Seconds.of(2);
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static final class AlgaeScorerConstants {
    public static final int PIVOT_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_ID = 8;

    public static final int PIVOT_ENCODER_A = 4;
    public static final int PIVOT_ENCODER_B = 5;

    public static final int ENCODER_RESOLUTION = 2048;

    public static final Angle PIVOT_STOW_ANGLE = Degrees.of(80);
    public static final Angle PIVOT_INTAKE_ANGLE = Degrees.of(30);

    public static double INTAKE_SPEED_PERCENT = 0.3;
    public static double OUTAKE_TAKE_SPEED_PERCENT = -0.4;

    public static final Mass ARM_MASS = Pounds.of(7.2172536);
    public static final Distance ARM_LENGTH = Inches.of(22.33);
    public static final MomentOfInertia ARM_MOI = KilogramSquareMeters.of(0.0715890237);

    public static final Current ROLLER_MOTOR_CURRENT_LIMIT = Amps.of(40);
    public static final Voltage ROLLER_MOTOR_VOLTAGE_COMP = Volts.of(12);
    public static final double GEARING = 42 / 22 * 45;
    public static final Angle PIVOT_ANGLE_DEADBAND = Degrees.of(2);

    public final class PivotPID {
      public static final double p = 0.015;
      public static final double i = 0.00;
      public static final double d = 0.00;
    }

    public final class PivotFeedForward {
      public static final double kg = 0.0001;
      public static final double kv = 0.0;
      public static final double ks = 0.0;
    }
  }
}
