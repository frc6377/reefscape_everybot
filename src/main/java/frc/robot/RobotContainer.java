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
import frc.robot.subsystems.CANdleSignalingSubsystem;
import java.lang.reflect.Method;
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
  public final CANdleSignalingSubsystem signalingSubsystem = new CANdleSignalingSubsystem();

  // The driver's controller
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Registering Auto Commands
  // The autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> hardAutoChooser;

  public static boolean coralMode = true;
  public boolean usingPP = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (AutoBuilder.isConfigured() && usingPP) {
      autoChooser = new LoggedDashboardChooser<>("PP Auto Choices", AutoBuilder.buildAutoChooser());
      hardAutoChooser = null;
    } else {
      autoChooser = null;
      hardAutoChooser = new SendableChooser<>();
      initializeHardAutos();
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
  public void PPAutoCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("ScoreCoralL1", coralScorerSubsystem.ejectCommand());

    NamedCommands.registerCommands(autonCommands);
  }

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
                  coralMode = !coralMode; // Toggle coralMode
                  Logger.recordOutput("Mode/Score Mode", coralMode ? "Coral Mode" : "Algae Mode");
                }));

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

    // Signaling Buttons
    driverController.y().onTrue(Commands.runOnce(() -> signalingSubsystem.setRandomAnimation()));
  }

  public double cubicCurve(DoubleSupplier input, double intensity) {
    return intensity * Math.pow(input.getAsDouble(), 3) + (1 - intensity) * input.getAsDouble();
  }

  private void initializeHardAutos() {
    if (hardAutoChooser != null) {
      for (Method method : Autos.class.getDeclaredMethods()) {
        if (method.getReturnType() == Command.class) {
          String name = method.getName();
          try {
            hardAutoChooser.addOption(
                beautifyName(name), (Command) method.invoke(null, driveSubsystem));
          } catch (Exception e) {
            DriverStation.reportError("Failed to add auto: " + name, true);
          }
        }
      }
      SmartDashboard.putData("HARD AUTO CHOOSER", hardAutoChooser);
    }
  }

  private String beautifyName(String camelCase) {
    String spaced =
        camelCase
            .replaceAll("([a-z])([A-Z])", "$1 $2")
            .replaceAll("([A-Z])([A-Z][a-z])", "$1 $2")
            .replaceAll("Auto$", "")
            .trim();

    return capitalize(spaced);
  }

  private String capitalize(String input) {
    if (input == null || input.isEmpty()) return input;
    return input.substring(0, 1).toUpperCase() + input.substring(1);
  }

  public static Command switchBotMode(boolean coralModebool) {
    return Commands.runOnce(() -> coralMode = coralModebool);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (usingPP) {
      if (autoChooser != null) {
        return autoChooser.get(); // Use AutoBuilder if configured
      } else {
        DriverStation.reportError(
            "AutoBuilder is not configured. Falling back to hard autos.", false);
        return Commands.none();
      }
    } else {
      Command selectedCommand = hardAutoChooser.getSelected();
      if (selectedCommand != null) {
        return selectedCommand;
      } else {
        DriverStation.reportError(
            "No hard auto selected. Defaulting to a 'do nothing' command.", true);
        return Commands.none();
      }
    }
  }
}
