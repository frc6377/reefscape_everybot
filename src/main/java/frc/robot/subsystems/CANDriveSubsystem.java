package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
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

  private double LeftSimMotorOutput;
  private double RightSimMotorOutput;

  private DifferentialDriveWheelSpeeds wheelSpeeds;

  private double leftEncoderRate = 0;
  private double rightEncoderRate = 0;

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

  private Field2d field = new Field2d();

  @SuppressWarnings("unused")
  private final DebugEntry<Field2d> fieldEntry = new DebugEntry<Field2d>(field, "FIELD", this);

  RobotConfig config;

  public CANDriveSubsystem() {
    field = new Field2d();

    // Create brushed motors for drive
    leftLeader = new VictorSPX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new VictorSPX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);

    leftEncoder =
        new Encoder(DriveConstants.LEFT_DRIVE_ENCODER_A, DriveConstants.LEFT_DRIVE_ENCODER_B);
    rightEncoder =
        new Encoder(DriveConstants.RIGHT_DRIVE_ENCODER_A, DriveConstants.RIGHT_DRIVE_ENCODER_B);

    leftEncoder.setDistancePerPulse(
        Math.PI * DriveConstants.WHEEL_DIAMETER_METERS / DriveConstants.ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(
        Math.PI * DriveConstants.WHEEL_DIAMETER_METERS / DriveConstants.ENCODER_RESOLUTION);

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);

    rightLeader.setInverted(false);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftLeader.setInverted(true);
    leftFollower.setInverted(InvertType.FollowMaster);

    gyro = new Pigeon2(DriveConstants.PIGEON_DEVICE_ID);
    gyroSim = new Pigeon2SimState(gyro);

    kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH_METERS);

    diffDrive =
        new DifferentialDrive(
            (speed) -> leftLeader.set(ControlMode.PercentOutput, speed),
            (speed) -> rightLeader.set(ControlMode.PercentOutput, speed));

    // Create drivetrain simulator
    diffDriveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getCIM(2),
            DriveConstants.GEARING,
            DriveConstants.MOI,
            DriveConstants.MASS_KILOGRAMS,
            DriveConstants.WHEEL_DIAMETER_METERS / 2,
            DriveConstants.TRACK_WIDTH_METERS,
            null);

    // Create new odometry object
    driveOdometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    defaultPathPlannerSetup();
  }

  public void defaultPathPlannerSetup() {
    try {
      config = RobotConfig.fromGUISettings();
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
            0.04), // PPLTVController is the built in path following controller for differential
        // drive trains
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

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
    leftEncoderSim.resetData();
    rightEncoderSim.resetData(); // Reset encoder values
    driveOdometry.resetPose(resetPose);
  }

  public Pose2d getPosition() {
    return position;
  }

  // Get Current Speed
  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void driveRobotRelative(ChassisSpeeds relativeSpeeds) {
    diffDrive.arcadeDrive(relativeSpeeds.vxMetersPerSecond, relativeSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    // Update position using odometry
    if (Robot.isReal()) {
      position =
          driveOdometry.update(
              gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

      wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    field.setRobotPose(position);

    leftEncoderRate = leftEncoder.getRate();
    rightEncoderRate = rightEncoder.getRate();
    gyroHeading = gyro.getRotation2d();

    leftEncoderEntry.log(leftEncoderRate);
    rightEncoderEntry.log(rightEncoderRate);
    gyroHeadingEntry.log(gyroHeading);
  }

  @Override
  public void simulationPeriodic() {
    // Simulate the motor inputs to the drivetrain
    diffDriveSim.setInputs(
        leftLeader.getMotorOutputPercent() * RobotController.getInputVoltage(),
        rightLeader.getMotorOutputPercent() * RobotController.getInputVoltage());

    SmartDashboard.putNumber(
        "leftSimMotorInput",
        leftLeader.getMotorOutputPercent() * RobotController.getInputVoltage());
    SmartDashboard.putNumber(
        "rightSimMotorInput",
        rightLeader.getMotorOutputPercent() * RobotController.getInputVoltage());

    // Update the simulation state
    diffDriveSim.update(0.02); // Update at 20ms intervals
    Logger.recordOutput("SimPose", diffDriveSim.getPose());

    // Update encoder and gyro states for simulation
    leftEncoderSim.setDistance(
        diffDriveSim.getLeftPositionMeters()); // Set distance from simulation
    leftEncoderSim.setRate(
        diffDriveSim.getLeftVelocityMetersPerSecond()); // Set rate from simulation

    SmartDashboard.putNumber("LeftEncoderSimSpeed", leftEncoderSim.getRate());

    rightEncoderSim.setDistance(
        diffDriveSim.getRightPositionMeters()); // Set distance from simulation
    rightEncoderSim.setRate(
        diffDriveSim.getRightVelocityMetersPerSecond()); // Set rate from simulation

    SmartDashboard.putNumber("RightEncoderSimSpeed", rightEncoderSim.getRate());

    gyroSim.setRawYaw(diffDriveSim.getHeading().getDegrees()); // Update simulated gyro
    Logger.recordOutput("Robot Position", driveOdometry.getPoseMeters());

    wheelSpeeds =
        new DifferentialDriveWheelSpeeds(leftEncoderSim.getRate(), rightEncoderSim.getRate());

    // Update odometry in simulation
    position =
        driveOdometry.update(
            gyro.getRotation2d(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
  }

  // Telemetry Commands
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(() -> diffDrive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()));
  }

  public Command stopRobotCommand() {
    return run(() -> diffDrive.arcadeDrive(0.0, 0.0));
  }
}
