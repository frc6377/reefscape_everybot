package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

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

  private final PIDController drivePID;
  private final PIDController rotatePID;

  private DifferentialDriveKinematics kinematics;

  private Pose2d position;

  private DifferentialDriveWheelSpeeds wheelSpeeds;

  private double leftEncoderRate = 0;
  private double rightEncoderRate = 0;

  private Rotation2d gyroHeading = new Rotation2d(0);

  // Simulation variables
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private Pigeon2SimState gyroSim;
  private DifferentialDrivetrainSim diffDriveSim;

  private Field2d field = new Field2d();

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
        Math.PI * DriveConstants.WHEEL_DIAMETER.in(Meter) / DriveConstants.ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(
        Math.PI * DriveConstants.WHEEL_DIAMETER.in(Meter) / DriveConstants.ENCODER_RESOLUTION);

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    rightLeader.setInverted(false);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    gyro = new Pigeon2(DriveConstants.PIGEON_DEVICE_ID);
    gyroSim = new Pigeon2SimState(gyro);

    kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH.in(Meter));

    driveModuleConfig =
        new ModuleConfig(
            DriveConstants.WHEEL_DIAMETER.in(Meter) / 2,
            DriveConstants.MAX_DRIVE_VELOCITY.in(MetersPerSecond),
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
              DriveConstants.MOI.in(KilogramSquareMeters),
              DriveConstants.MASS.in(Kilograms),
              DriveConstants.WHEEL_DIAMETER.in(Meter) / 2,
              DriveConstants.TRACK_WIDTH.in(Meter),
              null);
    }
    // Create new odometry object
    driveOdometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    rotatePID =
        new PIDController(
            DriveConstants.RotatePID.kp, DriveConstants.RotatePID.ki, DriveConstants.RotatePID.kd);
    drivePID =
        new PIDController(
            DriveConstants.DrivePID.kp, DriveConstants.DrivePID.ki, DriveConstants.DrivePID.kd);
  }

  public void defaultPathPlannerSetup() {
    try {
      config =
          new RobotConfig(
              DriveConstants.MASS.in(Kilograms),
              DriveConstants.MOI.in(KilogramSquareMeters),
              driveModuleConfig,
              DriveConstants.TRACK_WIDTH.in(Meter));
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPosition, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry
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
    Logger.recordOutput(
        "Drive/Forward Speed", kinematics.toChassisSpeeds(wheelSpeeds).vxMetersPerSecond);
    Logger.recordOutput(
        "Drive/Rot Speed - Radians", kinematics.toChassisSpeeds(wheelSpeeds).omegaRadiansPerSecond);
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
        -relativeSpeeds.vxMetersPerSecond / DriveConstants.MAX_DRIVE_VELOCITY.in(MetersPerSecond),
        relativeSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {

    wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    wheelSpeeds.desaturate(DriveConstants.MAX_DRIVE_VELOCITY);

    leftEncoderRate = leftEncoder.getRate();
    rightEncoderRate = rightEncoder.getRate();
    gyroHeading = gyro.getRotation2d();

    Logger.recordOutput("Drive/Left Encoder Speed", leftEncoderRate);
    Logger.recordOutput("Drive/Right EncoderSpeed", rightEncoderRate);
    Logger.recordOutput("Gyro Heading", gyroHeading);

    Logger.recordOutput(
        "Drive/leftMotorInput",
        leftLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    Logger.recordOutput(
        "Drive/rightMotorInput",
        rightLeader.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    Logger.recordOutput("Gyro Yaw", gyro.getYaw().getValue().in(Degrees));
    position =
        driveOdometry.update(
            gyro.getRotation2d(), -leftEncoder.getDistance(), -rightEncoder.getDistance());

    Logger.recordOutput("Robot Position", position);

    Logger.recordOutput(
        "Drive/Rotational Velocity(rads)",
        kinematics.toChassisSpeeds(wheelSpeeds).omegaRadiansPerSecond);
    Logger.recordOutput(
        "Drive/Linear Velocity(ms)", kinematics.toChassisSpeeds(wheelSpeeds).vxMetersPerSecond);
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
    return run(
        () -> {
          diffDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble());
          diffDrive.feed();
        });
  }

  public Command coralDrivetrain() {
    return run(
        () -> {
          leftLeader.setInverted(true);
          leftFollower.setInverted(InvertType.FollowMaster);
          rightLeader.setInverted(false);
          rightFollower.setInverted(InvertType.FollowMaster);
        });
  }

  public Command algaeDrivetrain() {
    return run(
        () -> {
          leftLeader.setInverted(false);
          leftFollower.setInverted(InvertType.FollowMaster);
          rightLeader.setInverted(true);
          rightFollower.setInverted(InvertType.FollowMaster);
        });
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

  public Command turnCommand(double targetAngle) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              rotatePID.setSetpoint(targetAngle);
              rotatePID.setTolerance(0.5);
              Logger.recordOutput("Drive/Rotate Target Angle", targetAngle);
            },
            this),
        run(() -> {
              double currentAngle = gyro.getYaw().getValue().in(Degrees);
              double PIDOutput = rotatePID.calculate(currentAngle, rotatePID.getSetpoint());
              Logger.recordOutput("Drive/Rotate Current Angle", currentAngle);
              Logger.recordOutput("Drive/Rotate PID Output", PIDOutput);

              diffDrive.arcadeDrive(0.0, PIDOutput);
            })
            .until(
                () -> {
                  double error = Math.abs(gyro.getYaw().getValue().in(Degrees) - targetAngle);
                  Logger.recordOutput("Rotate Error", error);
                  return rotatePID.atSetpoint()
                      && Math.abs(
                              RadiansPerSecond.of(
                                      kinematics.toChassisSpeeds(wheelSpeeds).omegaRadiansPerSecond)
                                  .in(DegreesPerSecond))
                          <= 0.0;
                }));
  }

  public Command driveCommand(double distance) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Pose2d currentPose = driveOdometry.getPoseMeters();
              Pose2d targetPose =
                  new Pose2d(
                      currentPose.getX() + distance, currentPose.getY(), currentPose.getRotation());
              drivePID.setSetpoint(targetPose.getX());
            },
            this),
        run(() -> {
              double PIDOutput =
                  drivePID.calculate(driveOdometry.getPoseMeters().getX(), drivePID.getSetpoint());
              Logger.recordOutput("Drive/Drive PID Output", PIDOutput);
              diffDrive.arcadeDrive(-PIDOutput, 0.0);
            })
            .until(
                () -> {
                  Pose2d currentPose = driveOdometry.getPoseMeters();
                  double error = Math.abs(drivePID.getSetpoint() - currentPose.getX());
                  return error == 0;
                }));
  }

  public Command goToRelativePose(Pose2d targetPose) {
    double xPose = targetPose.getX();
    double yPose = targetPose.getY();

    double zPose = Math.sqrt(Math.pow(xPose, 2) + Math.pow(yPose, 2));

    double angle = Math.atan2(yPose, xPose) - gyro.getRotation2d().getRadians();

    return Commands.sequence(
        turnCommand(Math.toDegrees(angle)),
        driveCommand(zPose),
        turnCommand(targetPose.getRotation().getDegrees()));
  }
}
