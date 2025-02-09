// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 1;
    public static final int RIGHT_FOLLOWER_ID = 2;

    public static final int RIGHT_DRIVE_ENCODER_A = 1;
    public static final int RIGHT_DRIVE_ENCODER_B = 2;

    public static final int LEFT_DRIVE_ENCODER_A = 5;
    public static final int LEFT_DRIVE_ENCODER_B = 6;

    public static final int PIGEON_DEVICE_ID = 0;

    public static final double TRACK_WIDTH_METERS = 0.57785;

    public static final double GEARING = 10.71;

    public static final double WHEEL_DIAMETER_METERS =
        0.1524; // FIXME: ADD CORRECT NUMBER DO THIS BEFORE DRIVING OR ROBOT GO KABOOM

    public static final double MAX_DRIVE_VELOCITY_MPS =
        12; // FIXME: ADD CORRECT NUMBER DO THIS BEFORE DRIVING OR ROBOT GO KABOOM

    public static final double WHEEL_COF =
        1.0; // FIXME: ADD CORRECT NUMBER DO THIS BEFORE DRIVING OR ROBOT GO KABOOM

    public static final double MOTOR_CURRENT_LIMIT =
        100000000; // FIXME: ADD CORRECT NUMBER DO THIS BEFORE DRIVING OR ROBOT GO KABOOM

    // COMPLETELY NOT SURE OF MOI
    public static final double MOI = 5.5;

    // MASS WITHOUT WIRING
    public static final double MASS_KILOGRAMS = 25.93153918200222;

    public static final double ENCODER_RESOLUTION = 8192;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 40;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 12;
    public static final double ROLLER_EJECT_PERCENT = 0.44;
    public static final double ROLLER_INTAKE_PERCENT = -0.44;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
