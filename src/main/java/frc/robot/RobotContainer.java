// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CANAlgaeManipulatorSubsystem;
import frc.robot.subsystems.CANCoralScorerSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  public final CANCoralScorerSubsystem coralScorerSubsystem = new CANCoralScorerSubsystem();
  public final CANAlgaeManipulatorSubsystem algaeScorerSubsystem =
      new CANAlgaeManipulatorSubsystem();
  public final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // The driver's controller
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Registering Auto Commands
  // The autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> hardAutoChooser;

  public static boolean visionPoseCorrection = false;
  public static boolean coralMode = true;
  public boolean usingPP = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (AutoBuilder.isConfigured()) {
      autoChooser = new LoggedDashboardChooser<>("PP Auto Choices", AutoBuilder.buildAutoChooser());
      hardAutoChooser = null;
      addPPAutoCommands();
    } else {
      autoChooser = null;
      hardAutoChooser = new SendableChooser<>();
      addHardAutos();
    }

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {

    // Coral Mode Buttons
    driverController
        .leftTrigger()
        .and(() -> coralMode)
        .whileTrue(coralScorerSubsystem.intakeCommand());
    driverController
        .rightTrigger()
        .and(() -> coralMode)
        .whileTrue(coralScorerSubsystem.ejectCommand());
    driverController
        .leftBumper()
        .and(() -> coralMode)
        .onTrue(coralScorerSubsystem.toggleIntakeSpeedCommand());

    // Algae Mode Buttons
    driverController
        .rightTrigger()
        .and(() -> !coralMode)
        .whileTrue(algaeScorerSubsystem.intakeAlgaeCommand());
    driverController
        .leftTrigger()
        .and(() -> !coralMode)
        .whileTrue(algaeScorerSubsystem.outakeAlgaeCommand());
    driverController
        .rightBumper()
        .and(() -> !coralMode)
        .onTrue(algaeScorerSubsystem.togglePivotCommand());

    // Mode Switching
    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralMode = !coralMode;
                }));

    driverController.b().onTrue(toggleVisionPoseCorrection());

    Logger.recordOutput("Mode/Score Mode", coralMode ? "Coral Mode" : "Algae Mode");
    driveSubsystem.setDefaultCommand(
        driveSubsystem.arcadeDrive(
            () -> coralMode ? driverController.getLeftY() : -driverController.getLeftY(),
            () -> driverController.getRightX()));

    driverController.povUp().whileTrue((driveSubsystem.driveCommand(1.00)));
    driverController
        .povRight()
        .whileTrue(Commands.runOnce(() -> driveSubsystem.turnCommand(45.00)));
    driverController
        .povDown()
        .whileTrue(
            Commands.runOnce(
                () ->
                    driveSubsystem.goToRelativePose(new Pose2d(0.5, 0.5, new Rotation2d(45.00)))));

    driverController.start().onTrue(Commands.run(() -> driveSubsystem.zeroPosition()));
  }

  public double cubicCurve(DoubleSupplier input, double intensity) {
    return intensity * Math.pow(input.getAsDouble(), 3) + (1 - intensity) * input.getAsDouble();
  }

  private void addHardAutos() {
    if (hardAutoChooser != null) {
      hardAutoChooser.addOption("Example Auto", Autos.exampleAuto(driveSubsystem));
      hardAutoChooser.addOption("Rotate Auto", Autos.rotateAuto(driveSubsystem));
      hardAutoChooser.addOption("Forward Auto", Autos.forwardAuto(driveSubsystem));
      SmartDashboard.putData("Hard Auto Chooser", hardAutoChooser);
    }
  }

  private void addPPAutoCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("ScoreCoralL1", coralScorerSubsystem.ejectCommand());

    NamedCommands.registerCommands(autonCommands);
  }

  public static Command switchBotMode(boolean coralModebool) {
    return Commands.runOnce(() -> coralMode = coralModebool);
  }

  public static Command toggleVisionPoseCorrection() {
    return Commands.runOnce(() -> visionPoseCorrection = !visionPoseCorrection);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (usingPP) {
      if (autoChooser != null) {
        return autoChooser.get();
      } else {
        DriverStation.reportError(
            "AutoBuilder is not configured. Falling back to hard autos.", false);
        return Commands.none();
      }
    } else if (hardAutoChooser != null) {
      Command selectedCommand = hardAutoChooser.getSelected();
      if (selectedCommand != null) {
        return selectedCommand;
      } else {
        DriverStation.reportError("No hard auto selected. Defaulting to Nothing Command", true);
        return Commands.none();
      }
    } else {
      DriverStation.reportError(
          "No AutoChooser has been set up. Defaulting to Nothing Command", true);
      return Commands.none();
    }
  }
}
