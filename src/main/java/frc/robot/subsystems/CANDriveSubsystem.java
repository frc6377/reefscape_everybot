package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import utilities.DebugEntry;

public class CANDriveSubsystem extends SubsystemBase {
  private final VictorSPX leftLeader;
  private final VictorSPX leftFollower;
  private final VictorSPX rightLeader;
  private final VictorSPX rightFollower;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private final Pigeon2 gyro;

  private final DifferentialDrive diffDrive;
  private final DifferentialDriveOdometry driveOdometry;

  private DifferentialDriveKinematics kinematics;

  private Pose2d position;

  private DifferentialDriveWheelSpeeds wheelSpeeds;

  private double leftEncoderRate = 0;
  private double rightEncoderRate = 0;

  private double leftEncoderRevo;
  private double rightEncoderRevo;

  private Rotation2d gyroHeading = new Rotation2d(0);

  // Simulation variables
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private Pigeon2SimState gyroSim;
  private DifferentialDrivetrainSim diffDriveSim;

  private DebugEntry<Double> leftEncoderEntry =
      new DebugEntry<Double>(leftEncoderRate, "Left Encoder Rate", this);
  private DebugEntry<Double> rightEncoderEntry =
      new DebugEntry<Double>(rightEncoderRate, "Right Encoder Rate", this);
  private DebugEntry<Rotation2d> gyroHeadingEntry =
      new DebugEntry<Rotation2d>(gyroHeading, "Gyro Heading", this);
  private DebugEntry<Double> leftEncoderRevoEntry =
      new DebugEntry<Double>(leftEncoderRevo, "LeftEncoderRevolutions", this);
  private DebugEntry<Double> rightEncoderRevoEntry =
      new DebugEntry<Double>(rightEncoderRevo, "RightEncoderRevolutions", this);

  private Field2d field = new Field2d();

  @SuppressWarnings("unused")
  private final DebugEntry<Field2d> fieldEntry = new DebugEntry<Field2d>(field, "FIELD", this);

  RobotConfig config;

  ModuleConfig driveModuleConfig;

  public CANDriveSubsystem() {
    field = new Field2d();

    // Create brushed motors for drive
    leftLeader = new VictorSPX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);

    leftEncoder =
        new Encoder(
            DriveConstants.LEFT_DRIVE_ENCODER_A, DriveConstants.LEFT_DRIVE_ENCODER_B, false);
    rightEncoder =
        new Encoder(
            DriveConstants.RIGHT_DRIVE_ENCODER_A, DriveConstants.RIGHT_DRIVE_ENCODER_B, true);

    leftEncoder.setDistancePerPulse(
        Math.PI
            * DriveConstants.WHEEL_DIAMETER_METERS.in(Meter)
            / DriveConstants.ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(
        Math.PI
            * DriveConstants.WHEEL_DIAMETER_METERS.in(Meter)
            / DriveConstants.ENCODER_RESOLUTION);

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);

    rightLeader.setInverted(true);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setInverted(false);
    leftFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    gyro = new Pigeon2(DriveConstants.PIGEON_DEVICE_ID);
    gyroSim = new Pigeon2SimState(gyro);

    kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS.in(Meter));

    driveModuleConfig =
        new ModuleConfig(
            DriveConstants.WHEEL_DIAMETER_METERS.in(Meter) / 2,
            DriveConstants.MAX_DRIVE_VELOCITY_MPS.in(MetersPerSecond),
            DriveConstants.WHEEL_COF,
            DCMotor.getCIM(2).withReduction(DriveConstants.GEARING),
            DriveConstants.MOTOR_CURRENT_LIMIT.in(Amps),
            2);

    diffDrive =
        new DifferentialDrive(
            (speed) -> leftLeader.set(ControlMode.PercentOutput, speed),
            (speed) -> rightLeader.set(ControlMode.PercentOutput, speed));

    // Create drivetrain simulator
    if (Robot.isSimulation()) {
      diffDriveSim =
          new DifferentialDrivetrainSim(
              DCMotor.getCIM(2),
              DriveConstants.GEARING,
              DriveConstants.MOI.in(KilogramMetersSquaredPerSecond),
              DriveConstants.MASS.in(Kilograms),
              DriveConstants.WHEEL_DIAMETER_METERS.in(Meter) / 2,
              DriveConstants.TRACK_WIDTH_METERS.in(Meter),
              null);
    }
    // Create new odometry object
    driveOdometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    defaultPathPlannerSetup();
  }

  public void defaultPathPlannerSetup() {
    try {
      config =
          new RobotConfig(
              DriveConstants.MASS.in(Kilograms),
              DriveConstants.MOI.in(KilogramMetersSquaredPerSecond),
              driveModuleConfig,
              DriveConstants.TRACK_WIDTH_METERS.in(Meter));
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPosition, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPLTVController(
            0.02), // PPLTVController is the built in path following controller for differential
        // drive trains
        config, // The robot configuration
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  // Pigeon Functions for Odometry
  public void resetOdometry(Pose2d resetPose) {
    leftEncoder.reset();
    rightEncoder.reset();
    driveOdometry.resetPose(resetPose);
  }

  public Pose2d getPosition() {
    return position;
  }

  // Get Current Speed
  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void setGyroRotation(double Rotation) {
    gyro.setYaw(Rotation);
  }

  public void zeroPosition() {
    Rotation2d zeroRotation = new Rotation2d(0);
    Pose2d zeroPose = new Pose2d(0.0, 0.0, zeroRotation);
    driveOdometry.resetPose(zeroPose);
  }

  public void driveRobotRelative(ChassisSpeeds relativeSpeeds) {
    diffDrive.arcadeDrive(
        relativeSpeeds.vxMetersPerSecond
            / DriveConstants.MAX_DRIVE_VELOCITY_MPS.in(MetersPerSecond),
        relativeSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber(
        "ForwardInput",
        relativeSpeeds.vxMetersPerSecond
            / DriveConstants.MAX_DRIVE_VELOCITY_MPS.in(MetersPerSecond));

    SmartDashboard.putNumber("PathPlannerForwardInput", relativeSpeeds.vxMetersPerSecond);
  }

  @Override
  public void periodic() {

    wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    wheelSpeeds.desaturate(DriveConstants.MAX_DRIVE_VELOCITY_MPS);

    leftEncoderRate = leftEncoder.getRate();
    rightEncoderRate = rightEncoder.getRate();
    gyroHeading = gyro.getRotation2d();

    leftEncoderEntry.log(leftEncoderRate);
    rightEncoderEntry.log(rightEncoderRate);
    gyroHeadingEntry.log(gyroHeading);

    leftEncoderRevoEntry.log((double) leftEncoder.getRaw() / 2048);
    rightEncoderRevoEntry.log((double) rightEncoder.getRaw() / 2048);

    SmartDashboard.putNumber(
        "leftMotorInput", leftLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber(
        "rightMotorInput",
        rightLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Gyro", gyro.getAccumGyroZ().getValue().in(Degrees));
    position =
        driveOdometry.update(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    Logger.recordOutput("Robot Position", position);
  }

  @Override
  public void simulationPeriodic() {
    // Simulate the motor inputs to the drivetrain
    diffDriveSim.setInputs(
        leftLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
        rightLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage());

    field.setRobotPose(position);

    // Update the simulation state
    diffDriveSim.update(0.02);
    Logger.recordOutput("SimPose", diffDriveSim.getPose());

    // Update encoder and gyro states for simulation
    leftEncoderSim.setDistance(
        diffDriveSim.getLeftPositionMeters()); // Set distance from simulation
    leftEncoderSim.setRate(
        diffDriveSim.getLeftVelocityMetersPerSecond()); // Set rate from simulation

    rightEncoderSim.setDistance(
        diffDriveSim.getRightPositionMeters()); // Set distance from simulation
    rightEncoderSim.setRate(
        diffDriveSim.getRightVelocityMetersPerSecond()); // Set rate from simulation

    gyroSim.setRawYaw(diffDriveSim.getHeading().getDegrees()); // Update simulated gyro
  }

  // Telemetry Commands
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(() -> diffDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()));
  }

  public Command stopRobotCommand() {
    return run(() -> diffDrive.arcadeDrive(0.0, 0.0));
  }

  public Command zeroGyro() {
    return Commands.runOnce(() -> setGyroRotation(0.0), this);
  }

  public Command zeroOdometry() {
    return Commands.runOnce(() -> zeroPosition(), this);
  }

  // public Command resetPathPose{

  // }
}
