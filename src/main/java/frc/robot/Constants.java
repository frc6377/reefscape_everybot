// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

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

    public static final int RIGHT_DRIVE_ENCODER_A = 1;
    public static final int RIGHT_DRIVE_ENCODER_B = 2;

    public static final int LEFT_DRIVE_ENCODER_A = 5;
    public static final int LEFT_DRIVE_ENCODER_B = 6;

    public static final int PIGEON_DEVICE_ID = 5;

    public static final Distance TRACK_WIDTH_METERS = Meters.of(0.5715);

    public static final double GEARING = 8.45;

    public static final Distance WHEEL_DIAMETER_METERS = Meters.of(0.15240);

    public static final LinearVelocity MAX_DRIVE_VELOCITY_MPS = MetersPerSecond.of(4.5);

    public static AngularVelocity MAX_RADIANS_PER_SECOND = RadiansPerSecond.of(6.675);

    public static final double WHEEL_COF = 0.975;

    public static final Current MOTOR_CURRENT_LIMIT = Amps.of(40);

    public static final AngularMomentum MOI = KilogramMetersSquaredPerSecond.of(7.0986919264);

    public static final Mass MASS = Kilograms.of(48.885739278871);

    public static final double ENCODER_RESOLUTION = 2048;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final Current ROLLER_MOTOR_CURRENT_LIMIT = Amps.of(40);
    public static final Voltage ROLLER_MOTOR_VOLTAGE_COMP = Volts.of(12);
    public static final double ROLLER_EJECT_PERCENT = 0.44;
    public static final double ROLLER_INTAKE_PERCENT = -0.44;

    public static final Time EJECT_TIME = Seconds.of(2);
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
